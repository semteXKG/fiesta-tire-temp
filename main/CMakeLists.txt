idf_component_register(SRCS "main.c" "wlan.c" "mqttcomm.c" "mlx90640.c" "tire_temp.c" "tire_segment.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver mqtt nvs_flash esp_netif esp_event esp_wifi esp_driver_i2c)
