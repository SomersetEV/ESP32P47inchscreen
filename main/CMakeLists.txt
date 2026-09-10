idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_twai nvs_flash fatfs sdmmc esp_driver_sdmmc esp_timer bt driver)
