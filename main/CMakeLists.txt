idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES button easy_gpio lora oled log esp_timer)