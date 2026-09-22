idf_component_register(
    SRCS "main.c" "camera_core.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash esp_wifi esp_netif lwip esp_http_server driver
)
