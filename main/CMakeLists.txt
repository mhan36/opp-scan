idf_component_register(SRCS "interval-scan.c" "scan.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES esp_timer
                    INCLUDE_DIRS "include")
