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

/* for temp and humidity sensor */
#include "aht.h"
#include "bmp280.h"
#include "i2cdev.h"

/* server config */
#include "sensitive.h"

/* topics */
#include "mqtt_topics.h"

/* for posting to MQTT broker */
#define MAX_RETRY 10
#define PAYLOAD_SIZE 16
char payload_aht20_temp[16];
char payload_aht20_hum[16];
char payload_bm280_temp[16];
char payload_bm280_press[16];

/* SPS30 payload buffers */
char payload_sps30_pm1[16];
char payload_sps30_pm25[16];
char payload_sps30_pm4[16];
char payload_sps30_pm10[16];
char payload_sps30_nc05[16];
char payload_sps30_nc1[16];
char payload_sps30_nc25[16];
char payload_sps30_nc4[16];
char payload_sps30_nc10[16];
char payload_sps30_tps[16];

/* I2C pins + speed */
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 100000

/* Sensor addresses */
#define AHT20_ADDR AHT_I2C_ADDRESS_GND
#define BMP280_ADDR BMP280_I2C_ADDRESS_1
#define SPS30_I2C_ADDR 0x69

/* SPS30 I2C commands (16-bit) */
#define SPS30_CMD_START_MEASUREMENT 0x0010
#define SPS30_CMD_STOP_MEASUREMENT 0x0104
#define SPS30_CMD_READ_DATA_READY 0x0202
#define SPS30_CMD_READ_MEASUREMENT 0x0300

static const char *TAG = "ESP32_SENSOR";

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;
esp_mqtt_client_handle_t client = NULL;

static aht_t aht;
static bmp280_t bmp;
static bool is_bme280 = false;

/* SPS30 i2c device descriptor */
static i2c_dev_t sps30_dev;

/* ===================== SPS30 FUNCTIONS ===================== */

/* CRC8: polynomial 0x31, init 0xFF (Sensirion standard) */
static uint8_t sensirion_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t sps30_write_cmd_with_args(uint16_t cmd,
                                           const uint8_t *args_words_crc,
                                           size_t args_len) {
    uint8_t buf[2 + 16] = {0};
    if (args_len > 16) {
        return ESP_ERR_INVALID_SIZE;
    }

    buf[0] = (uint8_t)(cmd >> 8);
    buf[1] = (uint8_t)(cmd & 0xFF);
    if (args_words_crc && args_len) {
        memcpy(&buf[2], args_words_crc, args_len);
    }

    /* Use i2cdev instead of i2c_master_write_to_device */
    esp_err_t err = i2c_dev_write(&sps30_dev, NULL, 0, buf, 2 + args_len);
    return err;
}

static esp_err_t sps30_write_cmd(uint16_t cmd) {
    return sps30_write_cmd_with_args(cmd, NULL, 0);
}

static esp_err_t sps30_read_words_with_crc(uint16_t cmd, uint8_t *out,
                                           size_t out_len) {
    uint8_t c[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};

    /* Write command, then read response using i2cdev */
    esp_err_t err = i2c_dev_write(&sps30_dev, NULL, 0, c, 2);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    err = i2c_dev_read(&sps30_dev, NULL, 0, out, out_len);
    return err;
}

static esp_err_t sps30_start_measurement(void) {
    uint8_t word[3];
    word[0] = 0x03;
    word[1] = 0x00;
    word[2] = sensirion_crc8(word, 2);

    esp_err_t err = sps30_write_cmd_with_args(SPS30_CMD_START_MEASUREMENT, word,
                                            sizeof(word));
    vTaskDelay(pdMS_TO_TICKS(50));
    return err;
}

static esp_err_t sps30_stop_measurement(void) {
    esp_err_t err = sps30_write_cmd(SPS30_CMD_STOP_MEASUREMENT);
    vTaskDelay(pdMS_TO_TICKS(50));
    return err;
}

static esp_err_t sps30_read_data_ready(bool *ready) {
    uint8_t rx[3] = {0};
    esp_err_t err =
        sps30_read_words_with_crc(SPS30_CMD_READ_DATA_READY, rx, sizeof(rx));
    if (err != ESP_OK) {
        return err;
    }

    if (sensirion_crc8(rx, 2) != rx[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t val = ((uint16_t)rx[0] << 8) | rx[1];
    *ready = (val == 1);
    return ESP_OK;
}

static esp_err_t sps30_read_measurement(float *vals_10) {
    uint8_t rx[60] = {0};
    esp_err_t err =
        sps30_read_words_with_crc(SPS30_CMD_READ_MEASUREMENT, rx, sizeof(rx));
    if (err != ESP_OK) {
        return err;
    }

    /* Validate CRC per 2-byte word and reconstruct into 40-byte payload */
    uint8_t payload[40] = {0};
    int p = 0;
    for (int i = 0; i < 60; i += 3) {
        if (sensirion_crc8(&rx[i], 2) != rx[i + 2]) {
            return ESP_ERR_INVALID_CRC;
        }
        payload[p++] = rx[i];
        payload[p++] = rx[i + 1];
    }

    /* Convert big-endian bytes to float (IEEE754) */
    for (int i = 0; i < 10; i++) {
        uint32_t u = ((uint32_t)payload[i * 4 + 0] << 24) |
                     ((uint32_t)payload[i * 4 + 1] << 16) |
                     ((uint32_t)payload[i * 4 + 2] << 8) |
                     ((uint32_t)payload[i * 4 + 3] << 0);

        float f;
        memcpy(&f, &u, sizeof(f));
        vals_10[i] = f;
    }

    return ESP_OK;
}

/* ===================== SENSOR TASKS ===================== */

/* Task for AHT20 and BMP280 sensors */
static void aht20_bm280_task(void *pv) {
    float aht_temp = 0.0f, aht_rh = 0.0f;
    float bmp_temp = 0.0f, bmp_press = 0.0f, bmp_hum = 0.0f;

    while (1) {
        esp_err_t err_aht = aht_get_data(&aht, &aht_temp, &aht_rh);
        float *hum_ptr = is_bme280 ? &bmp_hum : NULL;
        esp_err_t err_bmp = bmp280_read_float(&bmp, &bmp_temp, &bmp_press, hum_ptr);

        snprintf(payload_aht20_temp, PAYLOAD_SIZE, "%.2f", aht_temp);
        snprintf(payload_aht20_hum, PAYLOAD_SIZE, "%.2f", aht_rh);
        snprintf(payload_bm280_temp, PAYLOAD_SIZE, "%.2f", bmp_temp);
        snprintf(payload_bm280_press, PAYLOAD_SIZE, "%.2f", bmp_press);

        if (err_aht == ESP_OK && err_bmp == ESP_OK) {
            if (is_bme280) {
                ESP_LOGI(TAG,
                 "AHT20: T=%.2f C RH=%.2f %% | BME280: T=%.2f C P=%.2f Pa "
                 "RH=%.2f %%",
                 aht_temp, aht_rh, bmp_temp, bmp_press, bmp_hum);
            } else {
                ESP_LOGI(TAG, "AHT20: T=%.2f C RH=%.2f %% | BMP280: T=%.2f C P=%.2f Pa",
                 aht_temp, aht_rh, bmp_temp, bmp_press);

                esp_mqtt_client_publish(client, MQTT_TOPIC_AHT20_TEMP, payload_aht20_temp, 0, 1, 0);
                esp_mqtt_client_publish(client, MQTT_TOPIC_AHT20_HUM, payload_aht20_hum, 0, 1, 0);
                esp_mqtt_client_publish(client, MQTT_TOPIC_BM280_TEMP, payload_bm280_temp, 0, 1, 0);
                esp_mqtt_client_publish(client, MQTT_TOPIC_BM280_PRESS, payload_bm280_press, 0, 1, 0);
            }
        } else {
            if (err_aht != ESP_OK) {
                ESP_LOGW(TAG, "AHT20 read failed: %s", esp_err_to_name(err_aht));
            }
            if (err_bmp != ESP_OK) {
                ESP_LOGW(TAG, "BMP280 read failed: %s", esp_err_to_name(err_bmp));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* Task for SPS30 particle sensor */
static void sps30_task(void *pv) {
    /* If sensor is in a weird state, stopping first is harmless */
    (void)sps30_stop_measurement();

    esp_err_t err = sps30_start_measurement();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPS30 start measurement failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "SPS30 measurement started.");

    while (1) {
        bool ready = false;
        err = sps30_read_data_ready(&ready);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPS30 data_ready read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!ready) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        float v[10] = {0};
        err = sps30_read_measurement(v);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPS30 measurement read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* Format values for MQTT */
        snprintf(payload_sps30_pm1, PAYLOAD_SIZE, "%.2f", v[0]);
        snprintf(payload_sps30_pm25, PAYLOAD_SIZE, "%.2f", v[1]);
        snprintf(payload_sps30_pm4, PAYLOAD_SIZE, "%.2f", v[2]);
        snprintf(payload_sps30_pm10, PAYLOAD_SIZE, "%.2f", v[3]);
        snprintf(payload_sps30_nc05, PAYLOAD_SIZE, "%.2f", v[4]);
        snprintf(payload_sps30_nc1, PAYLOAD_SIZE, "%.2f", v[5]);
        snprintf(payload_sps30_nc25, PAYLOAD_SIZE, "%.2f", v[6]);
        snprintf(payload_sps30_nc4, PAYLOAD_SIZE, "%.2f", v[7]);
        snprintf(payload_sps30_nc10, PAYLOAD_SIZE, "%.2f", v[8]);
        snprintf(payload_sps30_tps, PAYLOAD_SIZE, "%.2f", v[9]);

        ESP_LOGI(TAG,
                 "SPS30 - PM(ug/m3): PM1.0=%.2f PM2.5=%.2f PM4.0=%.2f PM10=%.2f | "
                 "NC(#/cm3): >0.5=%.2f >1.0=%.2f >2.5=%.2f >4.0=%.2f >10=%.2f | "
                 "TPS=%.2f um",
                 v[0], v[1], v[2], v[3],
                 v[4], v[5], v[6], v[7], v[8],
                 v[9]);

        /* Publish to MQTT */
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_PM1, payload_sps30_pm1, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_PM25, payload_sps30_pm25, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_PM4, payload_sps30_pm4, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_PM10, payload_sps30_pm10, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_NC05, payload_sps30_nc05, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_NC1, payload_sps30_nc1, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_NC25, payload_sps30_nc25, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_NC4, payload_sps30_nc4, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_NC10, payload_sps30_nc10, 0, 1, 0);
        esp_mqtt_client_publish(client, MQTT_TOPIC_SPS30_TPS, payload_sps30_tps, 0, 1, 0);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

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
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

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

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32,
           base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
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
    snprintf(uri_string, sizeof(uri_string), "mqtt://%s", SERVER_IP);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri_string,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);
    esp_mqtt_client_start(client);
}

/* ================== MAIN APP ================== */

void app_main(void) {
    /* Initialize NVS (required for WiFi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize WiFi in station mode and connect */
    wifi_init_sta();

    /* Start MQTT */
    mqtt_app_start();

    /* Init i2cdev (shared by all I2C devices) */
    ESP_ERROR_CHECK(i2cdev_init());

    /* Init AHT20 (esp-idf-lib/aht) */
    memset(&aht, 0, sizeof(aht));
    ESP_ERROR_CHECK(
      aht_init_desc(&aht, AHT20_ADDR, I2C_PORT, I2C_SDA_GPIO, I2C_SCL_GPIO));
    aht.type = AHT_TYPE_AHT20;
    aht.mode = AHT_MODE_NORMAL;

    aht.i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
    aht.i2c_dev.cfg.sda_pullup_en = true;
    aht.i2c_dev.cfg.scl_pullup_en = true;

    ESP_ERROR_CHECK(aht_init(&aht));

    /* Init BMP280/BME280 (esp-idf-lib/bmp280) */
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

    /* Init SPS30 using i2cdev */
    memset(&sps30_dev, 0, sizeof(sps30_dev));
    ESP_ERROR_CHECK(i2c_dev_create_mutex(&sps30_dev));

    sps30_dev.port = I2C_PORT;
    sps30_dev.addr = SPS30_I2C_ADDR;
    sps30_dev.cfg.sda_io_num = I2C_SDA_GPIO;
    sps30_dev.cfg.scl_io_num = I2C_SCL_GPIO;
    sps30_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
    sps30_dev.cfg.sda_pullup_en = true;
    sps30_dev.cfg.scl_pullup_en = true;

    ESP_LOGI(TAG, "I2C init OK (SDA=%d, SCL=%d, freq=%d Hz)", I2C_SDA_GPIO,
           I2C_SCL_GPIO, I2C_FREQ_HZ);

    /* Create sensor tasks */
    xTaskCreate(aht20_bm280_task, "aht20_bm280_task", 4096, NULL, 5, NULL);
    xTaskCreate(sps30_task, "sps30_task", 4096, NULL, 5, NULL);
}
