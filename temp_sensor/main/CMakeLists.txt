idf_component_register(
    SRCS "bmx280.c" "main.c" "dht11.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
