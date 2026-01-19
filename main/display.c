#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"

static const char *TAG = "DisplayManager";

/* Display modes */
typedef enum {
    DISPLAY_MODE_MULTI_SENSOR,
    DISPLAY_MODE_GRAPH,
    DISPLAY_MODE_MAX
} display_mode_t;

/* Display context */
typedef struct {
    lv_obj_t *screen;
    lv_obj_t *label_title;
    lv_obj_t *label_value1;
    lv_obj_t *label_value2;
    lv_obj_t *label_value3;
    lv_obj_t *chart;
    lv_chart_series_t *chart_series;
    display_mode_t current_mode;
} display_ctx_t;

static display_ctx_t display_ctx;

/*
   Simulated sensor data (replace with your actual sensor readings)
   If more sensors need to be added, make a new float and add it in the appropried display modes(or just ask Ai to do it)
 */
static float get_sensor_temperature(void) {
    return 25.5f + (esp_random() % 100) / 10.0f;
}

static float get_sensor_humidity(void) {
    return 45.0f + (esp_random() % 200) / 10.0f;
}

static float get_sensor_pressure(void) {
    return 1013.0f + (esp_random() % 100) / 10.0f;
}

/* Create display mode: Single sensor view */
static void create_sensor_single_view(lv_obj_t *screen, const char *title) {
    lv_obj_clean(screen);

    /* Title */
    display_ctx.label_title = lv_label_create(screen);
    lv_label_set_text(display_ctx.label_title, title);
    lv_obj_set_style_text_color(display_ctx.label_title, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_title, LV_ALIGN_TOP_MID, 0, 10);

    /* Value */
    display_ctx.label_value1 = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_value1, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_value1, LV_ALIGN_CENTER, 0, 0);
}

/* Create display mode: Multiple sensors */
static void create_multi_sensor_view(lv_obj_t *screen) {
    lv_obj_clean(screen);

    /* Title */
    display_ctx.label_title = lv_label_create(screen);
    lv_label_set_text(display_ctx.label_title, "All Sensors");
    lv_obj_set_style_text_color(display_ctx.label_title, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_title, LV_ALIGN_TOP_MID, 0, 5);

    /* Temperature */
    display_ctx.label_value1 = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_value1, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_value1, LV_ALIGN_TOP_LEFT, 10, 35);

    /* Humidity */
    display_ctx.label_value2 = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_value2, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_value2, LV_ALIGN_TOP_LEFT, 10, 65);

    /* Pressure */
    display_ctx.label_value3 = lv_label_create(screen);
    lv_obj_set_style_text_color(display_ctx.label_value3, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_value3, LV_ALIGN_TOP_LEFT, 10, 95);
}

/* Create display mode: Graph view */
static void create_graph_view(lv_obj_t *screen) {
    lv_obj_clean(screen);

    /* Title */
    display_ctx.label_title = lv_label_create(screen);
    lv_label_set_text(display_ctx.label_title, "Temperature");
    lv_obj_set_style_text_color(display_ctx.label_title, lv_color_white(), 0);
    lv_obj_align(display_ctx.label_title, LV_ALIGN_TOP_MID, 0, 5);

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

/* Update display based on current mode */
static void update_display(void) {
    float temp = get_sensor_temperature();
    float humid = get_sensor_humidity();
    float press = get_sensor_pressure();

    char buf[64];

    switch (display_ctx.current_mode) {
    case DISPLAY_MODE_MULTI_SENSOR:
        snprintf(buf, sizeof(buf), "Temp: %.1f C", temp);
        lv_label_set_text(display_ctx.label_value1, buf);
        snprintf(buf, sizeof(buf), "Humid: %.1f %%", humid);
        lv_label_set_text(display_ctx.label_value2, buf);
        snprintf(buf, sizeof(buf), "Press: %.1f hPa", press);
        lv_label_set_text(display_ctx.label_value3, buf);
        break;

    case DISPLAY_MODE_GRAPH:
        lv_chart_set_next_value(display_ctx.chart, display_ctx.chart_series, (int16_t)temp);
        lv_chart_refresh(display_ctx.chart);
        break;

    default:
        break;
    }
}

/* Change display mode */
void display_set_mode(display_mode_t mode) {
    if (mode >= DISPLAY_MODE_MAX) {
        return;
    }

    ESP_LOGI(TAG, "Setting display mode to %d", mode);
    display_ctx.current_mode = mode;

    switch (mode) {
    case DISPLAY_MODE_MULTI_SENSOR:
        create_multi_sensor_view(display_ctx.screen);
        ESP_LOGI(TAG, "Created Multi-sensor view");
        break;

    case DISPLAY_MODE_GRAPH:
        create_graph_view(display_ctx.screen);
        ESP_LOGI(TAG, "Created Graph view");
        break;

    default:
        break;
    }

    update_display();
    ESP_LOGI(TAG, "Display updated");
}

/* Cycle through display modes (e.g., on button press) */
void display_next_mode(void) {
    display_mode_t next = (display_ctx.current_mode + 1) % DISPLAY_MODE_MAX;
    ESP_LOGI(TAG, "Switching to mode %d", next);
    display_set_mode(next);
}

/* Display update task */
static void display_task(void *pvParameter) {
    while (1) {
        /* Lock the LVGL mutex before any LVGL operations */
        if (bsp_display_lock(pdMS_TO_TICKS(10))) {
            update_display();
            lv_task_handler();
            bsp_display_unlock();
        }

        /* Yield to other tasks and watchdog */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* Initialize display */
void display_init(void) {
    ESP_LOGI(TAG, "Initializing display");

    /* Initialize BSP display */
    lv_display_t *disp = bsp_display_start();
    if (disp == NULL) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return;
    }

    /* Set backlight to maximum */
    ESP_LOGI(TAG, "Setting backlight");
    bsp_display_backlight_on();

    /* Small delay to let display stabilize */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Lock LVGL mutex before modifications */
    bsp_display_lock(0);

    /* Create screen */
    display_ctx.screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(display_ctx.screen, lv_color_black(), 0);
    lv_scr_load(display_ctx.screen);

    /* Set initial mode */
    display_ctx.current_mode = DISPLAY_MODE_MULTI_SENSOR;
    display_set_mode(DISPLAY_MODE_MULTI_SENSOR);

    /* Unlock LVGL mutex */
    bsp_display_unlock();

    ESP_LOGI(TAG, "Display initialized");
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Display Manager");

    /* Initialize display */
    display_init();

    /* Create display update task with larger stack */
    xTaskCreate(display_task, "display_task", 8192, NULL, 4, NULL);

    /* Example: Change mode every 5 seconds */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        /* Lock before changing mode */
        if (bsp_display_lock(pdMS_TO_TICKS(100))) {
            display_next_mode();
            bsp_display_unlock();
        }
    }
}
