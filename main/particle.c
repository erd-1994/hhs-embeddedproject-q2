#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "SPS30"

// I2C pins
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT I2C_NUM_0

#define SPS30_ADDR 0x69

// ---------------- CRC ----------------
static uint8_t sps30_crc(const uint8_t data[2])
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

// ---------------- Decode float ----------------
static float decode_float(const uint8_t *buf)
{
    uint8_t word1[2] = { buf[0], buf[1] };
    uint8_t crc1     = buf[2];
    uint8_t word2[2] = { buf[3], buf[4] };
    uint8_t crc2     = buf[5];

    if (sps30_crc(word1) != crc1 || sps30_crc(word2) != crc2) {
        ESP_LOGW(TAG, "CRC mismatch");
    }

    // SPS30 sends big endian float
    uint8_t be[4] = { word1[0], word1[1], word2[0], word2[1] };
    uint8_t le[4] = { be[3], be[2], be[1], be[0] }; // convert to little endian

    float f;
    memcpy(&f, le, sizeof(f));
    return f;
}

// ---------------- Start measurement ----------------
static esp_err_t sps30_start_measurement(void)
{
    uint8_t cmd[5];
    cmd[0] = 0x00;
    cmd[1] = 0x10;
    cmd[2] = 0x03;  // float format
    cmd[3] = 0x00;  
    uint8_t data[2] = {cmd[2], cmd[3]};
    cmd[4] = sps30_crc(data);

    return i2c_master_write_to_device(I2C_MASTER_PORT, SPS30_ADDR,
                                      cmd, sizeof(cmd),
                                      1000 / portTICK_PERIOD_MS);
}

// ---------------- Read measurement ----------------
static esp_err_t sps30_read_values(uint8_t *rxbuf)
{
    uint8_t cmd[2] = {0x03, 0x00};

    ESP_ERROR_CHECK(i2c_master_write_to_device(
        I2C_MASTER_PORT, SPS30_ADDR,
        cmd, sizeof(cmd), 1000 / portTICK_PERIOD_MS));

    vTaskDelay(pdMS_TO_TICKS(50));

    return i2c_master_read_from_device(
        I2C_MASTER_PORT, SPS30_ADDR,
        rxbuf, 60,
        1000 / portTICK_PERIOD_MS);
}

// ---------------- MAIN ----------------
void app_main(void)
{
    // ---- I2C init ----
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "I2C initialized");

    // ---- Start measurement ----
    ESP_ERROR_CHECK(sps30_start_measurement());
    ESP_LOGI(TAG, "SPS30 measurement started");

    vTaskDelay(pdMS_TO_TICKS(3000));

    uint8_t rxbuf[60];

    while (1) {

        if (sps30_read_values(rxbuf) == ESP_OK) {

            float pm1  = decode_float(&rxbuf[0]);
            float pm25 = decode_float(&rxbuf[6]);
            float pm4  = decode_float(&rxbuf[12]);
            float pm10 = decode_float(&rxbuf[18]);

            ESP_LOGI(TAG,
                "PM1.0: %.2f  | PM2.5: %.2f  | PM4: %.2f  | PM10: %.2f (µg/m³)",
                pm1, pm25, pm4, pm10);
        }
        else {
            ESP_LOGW(TAG, "Failed to read SPS30 data");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
