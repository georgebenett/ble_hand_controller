idf_component_register(
    SRCS "ble_spp_client.c" "main.c" "adc.c" "lcd.c"
    INCLUDE_DIRS "."
    REQUIRES driver 
            nvs_flash 
            bt
            esp_adc
            spi_flash
            esp_lcd
            lvgl
)
