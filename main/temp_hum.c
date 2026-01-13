// main.c
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "aht.h"
#include "bmp280.h"
#include "i2cdev.h"

// I2C pins + speed
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 100000 // 400000 also works

// Sensor addresses
#define AHT20_ADDR AHT_I2C_ADDRESS_GND   // AHT20 is typically 0x38
#define BMP280_ADDR BMP280_I2C_ADDRESS_1 // BMP280/BME280 is typically 0x76

// Tag for logging
static const char *TAG = "AHT20_BMP280";

// Global device objects
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

    if (err_aht == ESP_OK && err_bmp == ESP_OK) {
      if (is_bme280) {
        ESP_LOGI(TAG,
                 "AHT20: T=%.2f C RH=%.2f %% | BME280: T=%.2f C P=%.2f Pa "
                 "RH=%.2f %%",
                 aht_temp, aht_rh, bmp_temp, bmp_press, bmp_hum);
      } else {
        ESP_LOGI(TAG, "AHT20: T=%.2f C RH=%.2f %% | BMP280: T=%.2f C P=%.2f Pa",
                 aht_temp, aht_rh, bmp_temp, bmp_press);
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

void app_main(void) {
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

  // Start one task that reads both (simple + avoids I2C concurrency headaches)
  xTaskCreate(sensors_task, "sensors_task", 4096, NULL, 5, NULL);
}
