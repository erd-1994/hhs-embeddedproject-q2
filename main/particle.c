#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "SPS30";

// ===================== USER CONFIG =====================
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ 100000

#define SPS30_I2C_ADDR 0x69
// =======================================================

// SPS30 I2C commands (16-bit)
#define SPS30_CMD_START_MEASUREMENT 0x0010
#define SPS30_CMD_STOP_MEASUREMENT 0x0104
#define SPS30_CMD_READ_DATA_READY 0x0202
#define SPS30_CMD_READ_MEASUREMENT 0x0300

// CRC8: polynomial 0x31, init 0xFF (Sensirion standard)
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

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_GPIO,
      .scl_io_num = I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQ_HZ,
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
  return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static esp_err_t sps30_write_cmd_with_args(uint16_t cmd,
                                           const uint8_t *args_words_crc,
                                           size_t args_len) {
  // Build: [cmd_msb cmd_lsb] + args
  uint8_t buf[2 + 16] = {0};
  if (args_len > 16)
    return ESP_ERR_INVALID_SIZE;

  buf[0] = (uint8_t)(cmd >> 8);
  buf[1] = (uint8_t)(cmd & 0xFF);
  if (args_words_crc && args_len) {
    memcpy(&buf[2], args_words_crc, args_len);
  }

  return i2c_master_write_to_device(I2C_PORT, SPS30_I2C_ADDR, buf, 2 + args_len,
                                    pdMS_TO_TICKS(200));
}

static esp_err_t sps30_write_cmd(uint16_t cmd) {
  return sps30_write_cmd_with_args(cmd, NULL, 0);
}

static esp_err_t sps30_read_words_with_crc(uint16_t cmd, uint8_t *out,
                                           size_t out_len) {
  // Send cmd (2 bytes), then read out_len bytes
  uint8_t c[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};

  esp_err_t err = i2c_master_write_to_device(I2C_PORT, SPS30_I2C_ADDR, c, 2,
                                             pdMS_TO_TICKS(200));
  if (err != ESP_OK)
    return err;

  // SPS30 needs small processing time for some commands; safe tiny delay
  vTaskDelay(pdMS_TO_TICKS(5));

  err = i2c_master_read_from_device(I2C_PORT, SPS30_I2C_ADDR, out, out_len,
                                    pdMS_TO_TICKS(300));
  return err;
}

static esp_err_t sps30_start_measurement(void) {
  // For I2C: start measurement with arg = 0x0300 (float output)
  // Format: one 16-bit word + CRC
  uint8_t word[3];
  word[0] = 0x03;
  word[1] = 0x00;
  word[2] = sensirion_crc8(word, 2);

  esp_err_t err = sps30_write_cmd_with_args(SPS30_CMD_START_MEASUREMENT, word,
                                            sizeof(word));
  // SPS30 spec suggests wait a bit after starting
  vTaskDelay(pdMS_TO_TICKS(50));
  return err;
}

static esp_err_t sps30_stop_measurement(void) {
  esp_err_t err = sps30_write_cmd(SPS30_CMD_STOP_MEASUREMENT);
  vTaskDelay(pdMS_TO_TICKS(50));
  return err;
}

static esp_err_t sps30_read_data_ready(bool *ready) {
  // Response: 1 word + CRC = 3 bytes
  uint8_t rx[3] = {0};
  esp_err_t err =
      sps30_read_words_with_crc(SPS30_CMD_READ_DATA_READY, rx, sizeof(rx));
  if (err != ESP_OK)
    return err;

  if (sensirion_crc8(rx, 2) != rx[2])
    return ESP_ERR_INVALID_CRC;

  uint16_t val = ((uint16_t)rx[0] << 8) | rx[1];
  *ready = (val == 1);
  return ESP_OK;
}

static esp_err_t sps30_read_measurement(float *vals_10) {
  // SPS30 measurement: 10 floats = 40 bytes raw
  // Each float is 4 bytes, transferred as 2 words:
  // (2 bytes + crc) + (2 bytes + crc) => 6 bytes per float => 60 bytes total
  uint8_t rx[60] = {0};
  esp_err_t err =
      sps30_read_words_with_crc(SPS30_CMD_READ_MEASUREMENT, rx, sizeof(rx));
  if (err != ESP_OK)
    return err;

  // Validate CRC per 2-byte word and reconstruct into 40-byte payload
  uint8_t payload[40] = {0};
  int p = 0;
  for (int i = 0; i < 60; i += 3) {
    if (sensirion_crc8(&rx[i], 2) != rx[i + 2])
      return ESP_ERR_INVALID_CRC;
    payload[p++] = rx[i];
    payload[p++] = rx[i + 1];
  }

  // Convert big-endian bytes to float (IEEE754)
  // payload is 10 floats, each 4 bytes big-endian
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

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C init OK (SDA=%d, SCL=%d, freq=%d Hz)", I2C_SDA_GPIO,
           I2C_SCL_GPIO, I2C_FREQ_HZ);

  // If sensor is in a weird state, stopping first is harmless
  (void)sps30_stop_measurement();

  ESP_ERROR_CHECK(sps30_start_measurement());
  ESP_LOGI(TAG, "SPS30 measurement started.");

  while (1) {
    bool ready = false;
    esp_err_t err = sps30_read_data_ready(&ready);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "data_ready read failed: %s", esp_err_to_name(err));
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
      ESP_LOGE(TAG, "measurement read failed: %s", esp_err_to_name(err));
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // we don't need this much data
    // we only need PM1.0 PM2.5

    // Values:
    // v[0]  PM1.0 (ug/m3)
    // v[1]  PM2.5 (ug/m3)
    // v[2]  PM4.0 (ug/m3)
    // v[3]  PM10  (ug/m3)
    // v[4]  NC0.5 (#/cm3)
    // v[5]  NC1.0 (#/cm3)
    // v[6]  NC2.5 (#/cm3)
    // v[7]  NC4.0 (#/cm3)
    // v[8]  NC10  (#/cm3)
    // v[9]  Typical particle size (um)
    // ESP_LOGI(TAG,
    //          "PM(ug/m3): PM1.0=%.2f PM2.5=%.2f PM4.0=%.2f PM10=%.2f | "
    //          "NC(#/cm3): >0.5=%.2f >1.0=%.2f >2.5=%.2f >4.0=%.2f >10=%.2f | "
    //          "TPS=%.2f um",
    //          v[0], v[1], v[2], v[3],
    //          v[4], v[5], v[6], v[7], v[8],
    //          v[9]);
    ESP_LOGI(TAG, "PM(ug/m3): PM2.5=%.2f PM10=%.2f", v[1], v[3]);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
