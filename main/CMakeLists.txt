idf_component_register(SRCS "particle.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES 
		       esp_lcd
		       esp_timer
		       driver
		       esp_wifi
		       nvs_flash
		       mqtt
                       INCLUDE_DIRS "."
                       )
