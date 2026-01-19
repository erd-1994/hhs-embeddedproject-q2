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
#include "driver/gpio.h"

/* for temp and humidity sensor */
#include "aht.h"
#include "bmp280.h"
#include "i2cdev.h"

/* LVGL and display */
#include "lvgl.h"
#include "bsp/esp-bsp.h"

/* server config */
#include "sensitive.h"

/* topics */
#include "mqtt_topics.h"

/* ==================== ALARM CONFIGURATION ==================== */
#define BUZZER_PIN GPIO_NUM_33
#define TEMPERATURE_PIN GPIO_NUM_25
#define HUMIDITY_PIN GPIO_NUM_26
#define PARTICLE_PIN GPIO_NUM_27
#define LED_BLINK_DURATION 2000

/* Threshold values - adjust these as needed */
#define TEMP_HIGH_THRESHOLD 30.0f
#define TEMP_LOW_THRESHOLD 15.0f
#define HUM_HIGH_THRESHOLD 70.0f
#define HUM_LOW_THRESHOLD 30.0f
#define PM25_HIGH_THRESHOLD 35.0f  // WHO guideline: 25 μg/m³ daily, 35 is concerning

/* ==================== GLOBAL VARIABLES ==================== */
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

/* Shared sensor values for display */
static float g_aht_temp = 0.0f;
static float g_aht_rh = 0.0f;
static float g_bmp_temp = 0.0f;
static float g_bmp_press = 0.0f;
static float g_pm25 = 0.0f;
static float g_pm10 = 0.0f;
static bool g_mqtt_connected = false;

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

/* ==================== DISPLAY CONTEXT ==================== */
typedef enum {
    DISPLAY_MODE_MULTI_SENSOR,
    DISPLAY_MODE_GRAPH,
    DISPLAY_MODE_MAX
} display_mode_t;

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *label_title;
    lv_obj_t *label_temp;
    lv_obj_t *label_hum;
    lv_obj_t *label_press;
    lv_obj_t *label_pm25;
    lv_obj_t *label_status;
    lv_obj_t *chart;
    lv_chart_series_t *chart_series;
    display_mode_t current_mode;
} display_ctx_t;

static display_ctx_t display_ctx;

/* ===================== ALARM FUNCTIONS ===================== */

typedef enum {
    BUZZER,
    TEMPERATURE,
    HUMIDITY,
    PARTICLE
} AlarmType;

static void config_alarm_pins(void) {
    gpio_config_t buzz_conf = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&buzz_conf);

    gpio_config_t temp_conf = {
        .pin_bit_mask = (1ULL << TEMPERATURE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&temp_conf);

    gpio_config_t hum_conf = {
        .pin_bit_mask = (1ULL << HUMIDITY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&hum_conf);

    gpio_config_t particle_conf = {
        .pin_bit_mask = (1ULL << PARTICLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&particle_conf);

    gpio_set_level(BUZZER_PIN, 0);
    gpio_set_level(TEMPERATURE_PIN, 0);
    gpio_set_level(HUMIDITY_PIN, 0);
    gpio_set_level(PARTICLE_PIN, 0);

    ESP_LOGI(TAG, "Alarm pins configured");
}

static void trigger_alarm(AlarmType alarm_type, bool active) {
    gpio_num_t pin;
    const char *name;

    switch (alarm_type) {
    case BUZZER:
        pin = BUZZER_PIN;
        name = "BUZZER";
        break;
    case TEMPERATURE:
        pin = TEMPERATURE_PIN;
        name = "TEMPERATURE";
        break;
    case HUMIDITY:
        pin = HUMIDITY_PIN;
        name = "HUMIDITY";
        break;
    case PARTICLE:
        pin = PARTICLE_PIN;
        name = "PARTICLE";
        break;
    default:
        ESP_LOGW(TAG, "Unknown alarm type");
        return;
    }

    gpio_set_level(pin, active ? 1 : 0);
    if (active) {
        ESP_LOGW(TAG, "%s alarm TRIGGERED", name);
    }
}

static void check_alarms(void) {
    /* Temperature alarm */
    if (g_aht_temp > TEMP_HIGH_THRESHOLD || g_aht_temp < TEMP_LOW_THRESHOLD) {
        trigger_alarm(TEMPERATURE, true);
    } else {
        trigger_alarm(TEMPERATURE, false);
    }

    /* Humidity alarm */
    if (g_aht_rh > HUM_HIGH_THRESHOLD || g_aht_rh < HUM_LOW_THRESHOLD) {
        trigger_alarm(HUMIDITY, true);
    } else {
        trigger_alarm(HUMIDITY, false);
    }

    /* Particle alarm */
    if (g_pm25 > PM25_HIGH_THRESHOLD) {
        trigger_alarm(PARTICLE, true);
    } else {
        trigger_alarm(PARTICLE, false);
    }

    /* MQTT connection alarm */
    if (!g_mqtt_connected) {
        trigger_alarm(BUZZER, true);
    } else {
        trigger_alarm(BUZZER, false);
    }
}

/* ===================== DISPLAY FUNCTIONS ===================== */

static void create_multi_sensor_view(lv_obj_t *screen) {
    lv_obj_clean(screen);

    /* Title */
    display_ctx.label_title = lv_label_create(screen);
    lv_label_set_text(display_ctx.label_title, "Sensor Dashboard");
    lv_obj_set_style_text_color(display_ctx.label_title, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_title, LV_ALIGN_TOP_MID, 0, 5);

    /* Temperature */
    display_ctx.label_temp = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_temp, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_temp, LV_ALIGN_TOP_LEFT, 10, 30);

    /* Humidity */
    display_ctx.label_hum = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_hum, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_hum, LV_ALIGN_TOP_LEFT, 10, 55);

    /* Pressure */
    display_ctx.label_press = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_press, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_press, LV_ALIGN_TOP_LEFT, 10, 80);

    /* PM2.5 */
    display_ctx.label_pm25 = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_pm25, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_pm25, LV_ALIGN_TOP_LEFT, 10, 105);

    /* Status */
    // display_ctx.label_status = lv_label_create(screen);
    // lv_obj_set_style_text_color(display_ctx.label_status, lv_color_white(), 0);
    // lv_obj_align(display_ctx.label_status, LV_ALIGN_BOTTOM_MID, 0, -5);
}

static void create_graph_view(lv_obj_t *screen) {
    lv_obj_clean(screen);

    /* Title */
    display_ctx.label_title = lv_label_create(screen);
    lv_label_set_text(display_ctx.label_title, "Temperature Trend");
    lv_obj_set_style_text_color(display_ctx.label_title, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_title, LV_ALIGN_TOP_MID, 0, 5);

    /* Current value */
    display_ctx.label_temp = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_temp, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_temp, LV_ALIGN_TOP_MID, 0, 30);

    /* Chart */
    display_ctx.chart = lv_chart_create(screen);
    lv_obj_set_size(display_ctx.chart, 220, 100);
    lv_obj_align(display_ctx.chart, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_chart_set_type(display_ctx.chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(display_ctx.chart, 20);
    lv_chart_set_range(display_ctx.chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);

    display_ctx.chart_series = lv_chart_add_series(display_ctx.chart,
                                                     lv_palette_main(LV_PALETTE_RED),
                                                     LV_CHART_AXIS_PRIMARY_Y);
}

static void update_display(void) {
    char buf[64];

    switch (display_ctx.current_mode) {
    case DISPLAY_MODE_MULTI_SENSOR:
        snprintf(buf, sizeof(buf), "Temp: %.1f C", g_aht_temp);
        lv_label_set_text(display_ctx.label_temp, buf);

        snprintf(buf, sizeof(buf), "Humid: %.1f %%", g_aht_rh);
        lv_label_set_text(display_ctx.label_hum, buf);

        snprintf(buf, sizeof(buf), "PM2.5: %.1f ug/m3", g_pm25);
        lv_label_set_text(display_ctx.label_pm25, buf);

        snprintf(buf, sizeof(buf), "PM10: %.1f ug/m3", g_pm10);
        lv_label_set_text(display_ctx.label_press, buf);

        // snprintf(buf, sizeof(buf), "MQTT: %s", g_mqtt_connected ? "Connected" : "DISCONNECTED");
        // lv_label_set_text(display_ctx.label_status, buf);
        break;

    case DISPLAY_MODE_GRAPH:
        snprintf(buf, sizeof(buf), "%.1f C", g_aht_temp);
        lv_label_set_text(display_ctx.label_temp, buf);

        lv_chart_set_next_value(display_ctx.chart, display_ctx.chart_series, (int16_t)g_aht_temp);
        lv_chart_refresh(display_ctx.chart);
        break;

    default:
        break;
    }
}

static void display_set_mode(display_mode_t mode) {
    if (mode >= DISPLAY_MODE_MAX) {
        return;
    }

    ESP_LOGI(TAG, "Setting display mode to %d", mode);
    display_ctx.current_mode = mode;

    switch (mode) {
    case DISPLAY_MODE_MULTI_SENSOR:
        create_multi_sensor_view(display_ctx.screen);
        break;

    case DISPLAY_MODE_GRAPH:
        create_graph_view(display_ctx.screen);
        break;

    default:
        break;
    }

    update_display();
}

static void display_next_mode(void) {
    display_mode_t next = (display_ctx.current_mode + 1) % DISPLAY_MODE_MAX;
    display_set_mode(next);
}

static void display_task(void *pvParameter) {
    while (1) {
        //Display disabled - BSP not available
        if (bsp_display_lock(pdMS_TO_TICKS(10))) {
            update_display();
            lv_task_handler();
            bsp_display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void display_init(void) {
    ESP_LOGI(TAG, "Display disabled - BSP not configured");
    ESP_LOGI(TAG, "Initializing display");
    lv_display_t *disp = bsp_display_start();
    if (disp == NULL) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return;
    }
    bsp_display_backlight_on();
    vTaskDelay(pdMS_TO_TICKS(100));
    bsp_display_lock(0);
    display_ctx.screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(display_ctx.screen, lv_color_black(), 0);
    lv_scr_load(display_ctx.screen);
    display_ctx.current_mode = DISPLAY_MODE_MULTI_SENSOR;
    display_set_mode(DISPLAY_MODE_MULTI_SENSOR);
    bsp_display_unlock();
    ESP_LOGI(TAG, "Display initialized");
}

/* ===================== SPS30 FUNCTIONS ===================== */

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

    esp_err_t err = i2c_dev_write(&sps30_dev, NULL, 0, buf, 2 + args_len);
    return err;
}

static esp_err_t sps30_write_cmd(uint16_t cmd) {
    return sps30_write_cmd_with_args(cmd, NULL, 0);
}

static esp_err_t sps30_read_words_with_crc(uint16_t cmd, uint8_t *out,
                                           size_t out_len) {
    uint8_t c[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};

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

    uint8_t payload[40] = {0};
    int p = 0;
    for (int i = 0; i < 60; i += 3) {
        if (sensirion_crc8(&rx[i], 2) != rx[i + 2]) {
            return ESP_ERR_INVALID_CRC;
        }
        payload[p++] = rx[i];
        payload[p++] = rx[i + 1];
    }

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

static void aht20_bm280_task(void *pv) {
    float aht_temp = 0.0f, aht_rh = 0.0f;
    float bmp_temp = 0.0f, bmp_press = 0.0f, bmp_hum = 0.0f;

    while (1) {
        esp_err_t err_aht = aht_get_data(&aht, &aht_temp, &aht_rh);
        float *hum_ptr = is_bme280 ? &bmp_hum : NULL;
        esp_err_t err_bmp = bmp280_read_float(&bmp, &bmp_temp, &bmp_press, hum_ptr);

        if (err_aht == ESP_OK && err_bmp == ESP_OK) {
            /* Update global values */
            g_aht_temp = aht_temp;
            g_aht_rh = aht_rh;
            g_bmp_temp = bmp_temp;
            g_bmp_press = bmp_press;

            snprintf(payload_aht20_temp, PAYLOAD_SIZE, "%.2f", aht_temp);
            snprintf(payload_aht20_hum, PAYLOAD_SIZE, "%.2f", aht_rh);
            snprintf(payload_bm280_temp, PAYLOAD_SIZE, "%.2f", bmp_temp);
            snprintf(payload_bm280_press, PAYLOAD_SIZE, "%.2f", bmp_press);

            if (is_bme280) {
                ESP_LOGI(TAG,
                 "AHT20: T=%.2f C RH=%.2f %% | BME280: T=%.2f C P=%.2f Pa "
                 "RH=%.2f %%",
                 aht_temp, aht_rh, bmp_temp, bmp_press, bmp_hum);
            } else {
                ESP_LOGI(TAG, "AHT20: T=%.2f C RH=%.2f %% | BMP280: T=%.2f C P=%.2f Pa",
                 aht_temp, aht_rh, bmp_temp, bmp_press);

                if (g_mqtt_connected) {
                    esp_mqtt_client_publish(client, MQTT_TOPIC_AHT20_TEMP, payload_aht20_temp, 0, 1, 0);
                    esp_mqtt_client_publish(client, MQTT_TOPIC_AHT20_HUM, payload_aht20_hum, 0, 1, 0);
                    esp_mqtt_client_publish(client, MQTT_TOPIC_BM280_TEMP, payload_bm280_temp, 0, 1, 0);
                    esp_mqtt_client_publish(client, MQTT_TOPIC_BM280_PRESS, payload_bm280_press, 0, 1, 0);
                }
            }

            /* Check alarms after sensor update */
            check_alarms();
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

static void sps30_task(void *pv) {
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

        /* Update global values */
        g_pm25 = v[1];
        g_pm10 = v[3];

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

        /* Publish to MQTT if connected */
        if (g_mqtt_connected) {
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
        }

        /* Check alarms after sensor update */
        check_alarms();

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
        g_mqtt_connected = true;
        check_alarms();  /* Update alarm state */
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        g_mqtt_connected = false;
        check_alarms();  /* Trigger buzzer alarm */
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)",
               strerror(event->error_handle->esp_transport_sock_errno));
        }
        g_mqtt_connected = false;
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

/* ================== DISPLAY MODE SWITCHER TASK ================== */

static void display_mode_switcher_task(void *pv) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  /* Switch mode every 10 seconds */

       // Display disabled - BSP not available
        // if (bsp_display_lock(pdMS_TO_TICKS(100))) {
        //     display_next_mode();
        //     bsp_display_unlock();
        // }
    }
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

    /* Configure alarm pins */
    config_alarm_pins();

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

    /* Initialize display */
    display_init();

    /* Create sensor tasks */
    xTaskCreate(aht20_bm280_task, "aht20_bm280_task", 4096, NULL, 5, NULL);
    xTaskCreate(sps30_task, "sps30_task", 4096, NULL, 5, NULL);

    /* Create display update task */
    xTaskCreate(display_task, "display_task", 8192, NULL, 4, NULL);

    /* Create display mode switcher task */
    xTaskCreate(display_mode_switcher_task, "display_mode_switcher", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "System initialized successfully");
}