idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        mqtt
        driver
        esp_timer
        lvgl
        esp_lcd
        aht
        bmp280
        i2cdev
)
