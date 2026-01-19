#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_random.h"


#define MAX_RETRY       10

static const char *TAG = "ESP32_SENSOR2";

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static int s_retry_num = 0;
/* Global handle for the MQTT client so we can use it in the main loop */
esp_mqtt_client_handle_t client = NULL;

/* ================== WIFI EVENT HANDLER ================== */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data) {
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
    } else if (event_base == IP_EVENT &&
               event_id == IP_EVENT_STA_GOT_IP) {

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
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
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

    /* Wait for connection */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGI(TAG, "UNEXPECTED EVENT");
    }
}

/* ================== MQTT LOGIC ================== */

/* MQTT Event handler callback */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        /* If you wanted to subscribe to topics, you would do it here. */
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
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
        /*
           Depending on your ESP-IDF version, one of the following two lines is correct.
           For IDF v5.0+:
         */
        .broker.address.uri = uri_string,
        /*
           For IDF v4.x (Uncomment if using older version):
           .uri = uri_string,
         */
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    /* Register the MQTT event handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    /* Start the MQTT client */
    esp_mqtt_client_start(client);
}

/* ================== SENSOR LOGIC ================== */

/* Generate a pseudo-random temperature, e.g. between 18.0 and 30.0 */
static float generate_random_temperature(void) {
    uint32_t r = esp_random();  /* 0 .. 2^32-1 */
    float normalized = (float)(r % 1000) / 1000.0f; /* 0.000 .. 0.999 */
    float temp = 18.0f + normalized * 12.0f;        /* 18.0 .. 30.0 */
    return temp;
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

    /* Main Loop: Generate and Publish Temperature */
    char payload[20]; /* Buffer to hold the string representation of float */

    while (1) {
        /* 1. Get the temperature */
        float temp = generate_random_temperature();

        /* 2. Format into a string (e.g., "24.52") */
        snprintf(payload, sizeof(payload), "%.2f", temp);

        /*
           3. Publish to MQTT topic
           Parameters: client, topic, data, length (0 = calc from string), qos (0, 1, or 2), retain (0 or 1)
         */
        int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, payload, 0, 1, 0);

        if (msg_id != -1) {
            ESP_LOGI(TAG, "Sent Temperature: %s to topic %s", payload, MQTT_TOPIC);
        } else {
            ESP_LOGE(TAG, "Failed to publish message");
        }

        /* 4. Wait for 5 seconds before next reading */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
