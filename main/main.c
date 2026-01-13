#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

// for temp and humidity sensor
#include "aht.h"
#include "bmp280.h"
#include "i2cdev.h"

#include "sensitive.h"

// for posting to MQTT broker
#define MAX_RETRY 10
#define MQTT_TOPIC "/test/floatdata"
#define PAYLOAD_SIZE 200
char payload[200]; // Buffer to hold the string representation of float

// I2C pins + speed
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 100000 // 400000 also works

// Sensor addresses
#define AHT20_ADDR AHT_I2C_ADDRESS_GND   // AHT20 is typically 0x38
#define BMP280_ADDR BMP280_I2C_ADDRESS_1 // BMP280/BME280 is typically 0x76

static const char *TAG = "ESP32_SENSOR";

// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;
// Global handle for the MQTT client so we can use it in the main loop
esp_mqtt_client_handle_t client = NULL;

// TODO TEMP AND HUM CONFIG

static aht_t aht;
static bmp280_t bmp;           // Can be BMP280 or BME280
static bool is_bme280 = false; // Flag to indicate if BME280 is used

// Task that reads both sensors and logs data
static void sensors_task(void *pv) {
  float aht_temp = 0.0f, aht_rh = 0.0f;
  float bmp_temp = 0.0f, bmp_press = 0.0f, bmp_hum = 0.0f;

  while (1) {
    // --- AHT20 ---
    esp_err_t err_aht = aht_get_data(&aht, &aht_temp, &aht_rh);

    // --- BMP280/BME280 ---
    float *hum_ptr = is_bme280 ? &bmp_hum : NULL;
    esp_err_t err_bmp = bmp280_read_float(&bmp, &bmp_temp, &bmp_press, hum_ptr);

    snprintf(payload, PAYLOAD_SIZE,
             "AHT20: T=%.2f C RH=%.2f %% | BMP280: T=%.2f C P=%.2f Pa",
             aht_temp, aht_rh, bmp_temp, bmp_press);

    if (err_aht == ESP_OK && err_bmp == ESP_OK) {
      if (is_bme280) {
        ESP_LOGI(TAG,
                 "AHT20: T=%.2f C RH=%.2f %% | BME280: T=%.2f C P=%.2f Pa "
                 "RH=%.2f %%",
                 aht_temp, aht_rh, bmp_temp, bmp_press, bmp_hum);
      } else {
        // Post to mqtt
        ESP_LOGI(TAG, "AHT20: T=%.2f C RH=%.2f %% | BMP280: T=%.2f C P=%.2f Pa",
                 aht_temp, aht_rh, bmp_temp, bmp_press);
        esp_mqtt_client_publish(client, MQTT_TOPIC, payload, 0, 1, 0);
      }
    } else {
      if (err_aht != ESP_OK)
        ESP_LOGW(TAG, "AHT20 read failed: %s", esp_err_to_name(err_aht));
      if (err_bmp != ESP_OK)
        ESP_LOGW(TAG, "BMP280 read failed: %s", esp_err_to_name(err_bmp));
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// TODO WIFI AND MQTT CONFIG
//
/* ================== WIFI EVENT HANDLER ================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {

    if (s_retry_num < MAX_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retrying to connect to the AP...");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "Connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

/* ================== WIFI INITIALIZATION ================== */

static void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &instance_any_id));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  // Wait for connection
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to AP SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s password:%s", WIFI_SSID,
             WIFI_PASS);
  } else {
    ESP_LOGI(TAG, "UNEXPECTED EVENT");
  }
}

/* ================== MQTT LOGIC ================== */

// MQTT Event handler callback
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32,
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;

  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    // If you wanted to subscribe to topics, you would do it here.
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      ESP_LOGI(TAG, "Last errno string (%s)",
               strerror(event->error_handle->esp_transport_sock_errno));
    }
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

static void mqtt_app_start(void) {
  char uri_string[64];
  // Construct the full URI (e.g., mqtt://20.208.66.194)
  snprintf(uri_string, sizeof(uri_string), "mqtt://%s", SERVER_IP);

  esp_mqtt_client_config_t mqtt_cfg = {
      // Depending on your ESP-IDF version, one of the following two lines is
      // correct.
      // For IDF v5.0+:
      .broker.address.uri = uri_string,
      // For IDF v4.x (Uncomment if using older version):
      // .uri = uri_string,
  };

  client = esp_mqtt_client_init(&mqtt_cfg);

  // Register the MQTT event handler
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);

  // Start the MQTT client
  esp_mqtt_client_start(client);
}


/* ================== MAIN APP ================== */

void app_main(void) {
  // Initialize NVS (required for WiFi)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize WiFi in station mode and connect
  wifi_init_sta();

  // Start MQTT
  mqtt_app_start();

  // Init i2cdev (shared by both drivers)
  ESP_ERROR_CHECK(i2cdev_init());

  // Init AHT20 (esp-idf-lib/aht)
  memset(&aht, 0, sizeof(aht));
  ESP_ERROR_CHECK(
      aht_init_desc(&aht, AHT20_ADDR, I2C_PORT, I2C_SDA_GPIO, I2C_SCL_GPIO));
  aht.type = AHT_TYPE_AHT20;
  aht.mode = AHT_MODE_NORMAL;

  // Configure bus parameters (same pins/speed for all devices on this port)
  aht.i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
  aht.i2c_dev.cfg.sda_pullup_en = true; // Enable internal pull-ups
  aht.i2c_dev.cfg.scl_pullup_en = true;

  ESP_ERROR_CHECK(aht_init(&aht));

  // Init BMP280/BME280 (esp-idf-lib/bmp280)
  bmp280_params_t params;
  bmp280_init_default_params(&params);

  memset(&bmp, 0, sizeof(bmp));
  ESP_ERROR_CHECK(bmp280_init_desc(&bmp, BMP280_ADDR, I2C_PORT, I2C_SDA_GPIO,
                                   I2C_SCL_GPIO));

  bmp.i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;

  bmp.i2c_dev.cfg.sda_pullup_en = true;
  bmp.i2c_dev.cfg.scl_pullup_en = true;

  ESP_ERROR_CHECK(bmp280_init(&bmp, &params));

  is_bme280 = (bmp.id == BME280_CHIP_ID);
  ESP_LOGI(TAG, "Found %s (chip id=0x%02X)", is_bme280 ? "BME280" : "BMP280",
           bmp.id);

  // Main Loop: Generate and Publish Temperature

  xTaskCreate(sensors_task, "sensors_task", 4096, NULL, 5, NULL);
}


