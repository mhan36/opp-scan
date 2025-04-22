idf_component_register(SRCS "scan.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    INCLUDE_DIRS "")
