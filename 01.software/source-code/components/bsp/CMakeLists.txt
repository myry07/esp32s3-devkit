idf_component_register(SRCS "bsp.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_timer freertos
                    esp_driver_sdmmc
                    fatfs
                    sdmmc)
