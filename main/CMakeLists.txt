idf_component_register(SRCS
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "."
                       REQUIRES 
                          esp_lcd
                          esp_timer
                          driver
                          lvgl
                          esp_bsp_generic
                       )
