// main.c
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "AHT20_BMP280"

// I2C config
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

// Sensor addresses
#define AHT20_ADDR 0x38
#define BMP280_ADDR 0x76   

// BMP280 registers
#define BMP280_REG_CALIB 0x88
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7

typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

static bmp280_calib_t bmp280_calib;
static int32_t t_fine;

//  I2C helpers 
static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_write_bytes(uint8_t addr, const uint8_t *data, size_t len) {
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

//  AHT20 
static void aht20_init(void) {
    uint8_t cmd[3] = {0xBE, 0x08, 0x00};
    i2c_write_bytes(AHT20_ADDR, cmd, 3);
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void aht20_read(float *temp_c, float *hum_rh) {
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t buf[7];
    i2c_write_bytes(AHT20_ADDR, cmd, 3);
    vTaskDelay(pdMS_TO_TICKS(80));
    i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR, buf, 7, pdMS_TO_TICKS(100));

    uint32_t raw_h = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t raw_t = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    *hum_rh = (raw_h * 100.0f) / 1048576.0f;
    *temp_c = (raw_t * 200.0f) / 1048576.0f - 50.0f;
}

//  BMP280
static void bmp280_read_calib(void) {
    uint8_t c[24];
    i2c_read_reg(BMP280_ADDR, BMP280_REG_CALIB, c, 24);

    bmp280_calib.dig_T1 = (uint16_t)(c[1] << 8 | c[0]);
    bmp280_calib.dig_T2 = (int16_t)(c[3] << 8 | c[2]);
    bmp280_calib.dig_T3 = (int16_t)(c[5] << 8 | c[4]);
    bmp280_calib.dig_P1 = (uint16_t)(c[7] << 8 | c[6]);
    bmp280_calib.dig_P2 = (int16_t)(c[9] << 8 | c[8]);
    bmp280_calib.dig_P3 = (int16_t)(c[11] << 8 | c[10]);
    bmp280_calib.dig_P4 = (int16_t)(c[13] << 8 | c[12]);
    bmp280_calib.dig_P5 = (int16_t)(c[15] << 8 | c[14]);
    bmp280_calib.dig_P6 = (int16_t)(c[17] << 8 | c[16]);
    bmp280_calib.dig_P7 = (int16_t)(c[19] << 8 | c[18]);
    bmp280_calib.dig_P8 = (int16_t)(c[21] << 8 | c[20]);
    bmp280_calib.dig_P9 = (int16_t)(c[23] << 8 | c[22]);
}

static void bmp280_init(void) {
    bmp280_read_calib();
    uint8_t buf[2];
    buf[0] = BMP280_REG_CONFIG; buf[1] = 0x00; // no filter, default standby
    i2c_write_bytes(BMP280_ADDR, buf, 2);
    buf[0] = BMP280_REG_CTRL_MEAS; buf[1] = 0x27; // T x1, P x1, normal mode
    i2c_write_bytes(BMP280_ADDR, buf, 2);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void bmp280_read_raw(int32_t *temp_raw, int32_t *press_raw) {
    uint8_t d[6];
    i2c_read_reg(BMP280_ADDR, BMP280_REG_PRESS_MSB, d, 6);

    *press_raw = (int32_t)((((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4)));
    *temp_raw  = (int32_t)((((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4)));
}

static int32_t bmp280_compensate_T(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) * ((int32_t)bmp280_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8; // temperature x 100
}

static uint32_t bmp280_compensate_P(int32_t adc_P) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;

    if (var1 == 0) return 0; // avoid exception caused by division by zero
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var2 = ((int64_t)bmp280_calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    int64_t var3 = ((int64_t)bmp280_calib.dig_P8 * p) >> 19;
    p = ((p + var2 + var3) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
    return (uint32_t)p; // pressure in Pa
}

//  main 
void app_main(void) {
    i2c_master_init();
    aht20_init();
    bmp280_init();

    while (1) {
        float aht_temp, aht_hum;
        int32_t t_raw, p_raw;
        float bmp_temp_c;
        float bmp_press_hpa;

        aht20_read(&aht_temp, &aht_hum);
        bmp280_read_raw(&t_raw, &p_raw);
        int32_t t100 = bmp280_compensate_T(t_raw);
        uint32_t p_pa = bmp280_compensate_P(p_raw);

        bmp_temp_c = t100 / 100.0f;// Convert to °C
        bmp_press_hpa = p_pa / 100.0f;// Convert Pa to hPa

        ESP_LOGI(TAG, "AHT20: T=%.2f C, RH=%.2f %% | BMP280: T=%.2f C, P=%.2f hPa",
                 aht_temp, aht_hum, bmp_temp_c, bmp_press_hpa);// Log data 

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
