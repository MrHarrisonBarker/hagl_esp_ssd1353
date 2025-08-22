idf_component_register(
    SRCS "src/hagl_hal.c" "src/ssd1353.c"
    INCLUDE_DIRS "./include"
    REQUIRES hagl
    PRIV_REQUIRES driver esp_lcd
)