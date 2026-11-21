idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_rmt
                    REQUIRES button easy_gpio lora oled log esp_timer)