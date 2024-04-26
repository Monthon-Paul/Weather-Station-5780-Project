idf_component_register(
    SRCS "main.c" "lvgl_demo_ui.c" "bmx280.c" "dht11.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver esp_timer)
