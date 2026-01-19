#include "driver/gpio.h"       /* used for GPIO operations */
#include "freertos/FreeRTOS.h" /* used for FreeRTOS operations */
#include "freertos/task.h"     /* used for FreeRTOS tasks */
#include "esp_task_wdt.h"      /* used for Watchdog Timer operations */
#include "esp_log.h"           /* used for printing logging messages */

#define BUZZER_PIN GPIO_NUM_33
#define TEMPERATURE_PIN GPIO_NUM_25
#define HUMIDITY_PIN GPIO_NUM_26
#define PARTICLE_PIN GPIO_NUM_27

#define TAG "ALARM LED"         /* Tag for logging */
#define LED_BLINK_DURATION 2000        /* Tag for logging */

typedef enum
{
    BUZZER,
    TEMPERATURE,
    HUMIDITY,
    PARTICLE
} AlarmType;

void config_blink_led_alarm() {

    gpio_config_t buzz_conf = {
        .pin_bit_mask = (1ULL << TEMPERATURE_PIN), /* Pin assigned to buzzer */
        .mode = GPIO_MODE_OUTPUT,             /* Output mode */
        .pull_up_en = GPIO_PULLUP_DISABLE,    /* Disable pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* Disable pull-down */
        .intr_type = GPIO_INTR_DISABLE /* Disable interrupt */
    };
    gpio_config(&buzz_conf);

    gpio_config_t temp_conf = {
        .pin_bit_mask = (1ULL << TEMPERATURE_PIN), /* Pin assigned to temperature sensor */
        .mode = GPIO_MODE_OUTPUT,             /* Output mode */
        .pull_up_en = GPIO_PULLUP_DISABLE,    /* Disable pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* Disable pull-down */
        .intr_type = GPIO_INTR_DISABLE /* Disable interrupt */
    };
    gpio_config(&temp_conf);

    gpio_config_t hum_conf = {
        .pin_bit_mask = (1ULL << HUMIDITY_PIN), /* Pin assigned to humidity sensor */
        .mode = GPIO_MODE_OUTPUT,             /* Output mode */
        .pull_up_en = GPIO_PULLUP_DISABLE,    /* Disable pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* Disable pull-down */
        .intr_type = GPIO_INTR_DISABLE /* Disable interrupt */
    };
    gpio_config(&hum_conf);

    gpio_config_t particle_conf = {
        .pin_bit_mask = (1ULL << PARTICLE_PIN), /* Pin assigned to particle sensor */
        .mode = GPIO_MODE_OUTPUT,             /* Output mode */
        .pull_up_en = GPIO_PULLUP_DISABLE,    /* Disable pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* Disable pull-down */
        .intr_type = GPIO_INTR_DISABLE /* Disable interrupt */
    };
    gpio_config(&particle_conf);

    gpio_reset_pin(BUZZER_PIN);                       /* Reset the GPIO pin to default state */
    gpio_set_level(BUZZER_PIN, 0);                    /* Start with LED turned off */
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT); /* Set the GPIO pin as output */

    gpio_reset_pin(TEMPERATURE_PIN);                       /* Reset the GPIO pin to default state */
    gpio_set_level(TEMPERATURE_PIN, 0);                    /* Start with LED turned off */
    gpio_set_direction(TEMPERATURE_PIN, GPIO_MODE_OUTPUT); /* Set the GPIO pin as output */

    gpio_reset_pin(HUMIDITY_PIN);                       /* Reset the GPIO pin to default state */
    gpio_set_level(HUMIDITY_PIN, 0);                    /* Start with LED turned off */
    gpio_set_direction(HUMIDITY_PIN, GPIO_MODE_OUTPUT); /* Set the GPIO pin as output */

    gpio_reset_pin(PARTICLE_PIN);                       /* Reset the GPIO pin to default state */
    gpio_set_level(PARTICLE_PIN, 0);                    /* Start with LED turned off */
    gpio_set_direction(PARTICLE_PIN, GPIO_MODE_OUTPUT); /* Set the GPIO pin as output */


}

void blink_led_alarm(AlarmType alarm_type) {
    switch (alarm_type) {
    case BUZZER:
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DURATION));
        gpio_set_level(BUZZER_PIN, 0);
        break;
    case TEMPERATURE:
        gpio_set_level(TEMPERATURE_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DURATION));
        gpio_set_level(TEMPERATURE_PIN, 0);
        break;
    case HUMIDITY:
        gpio_set_level(HUMIDITY_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DURATION));
        gpio_set_level(HUMIDITY_PIN, 0);
        break;
    case PARTICLE:
        gpio_set_level(PARTICLE_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DURATION));
        gpio_set_level(PARTICLE_PIN, 0);
        break;
    default:
        ESP_LOGI(TAG, "Unknown alarm type.\n");
    }
}


void app_main() {
    ESP_LOGI(TAG, "Started %s Application", TAG);    /* Log the start of the application */
    esp_task_wdt_deinit();                         /* Disable the Watchdog */
    config_blink_led_alarm();

    while (1) {
        blink_led_alarm(TEMPERATURE);
        ESP_LOGI(TAG, "Toggled TEMPERATURE"); /* Log the start of the application */
        blink_led_alarm(HUMIDITY);
        ESP_LOGI(TAG, "Toggled HUMIDITY"); /* Log the start of the application */
        blink_led_alarm(PARTICLE);
        ESP_LOGI(TAG, "Toggled PARTICLE"); /* Log the start of the application */
        blink_led_alarm(BUZZER);
        ESP_LOGI(TAG, "Toggled BUZZER"); /* Log the start of the application */
        vTaskDelay(pdMS_TO_TICKS(5000)); /* easy on the CPU */
    }
}
