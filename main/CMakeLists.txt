idf_component_register(
    SRCS "main.c" "can_decode.c" "vehicle_state.c" "rtc_time.c"
         "can_handler.c" "can_logger.c" "sd_store.c" "sd_host_init_shim.c"
         "ble_nus.c"
         "ui/ui.c" "ui/ui_splash.c" "ui/ui_dash.c" "ui/sim_can.c"
    INCLUDE_DIRS "." "ui"
    EMBED_FILES "assets/logo.png" "assets/Montserrat-Medium.ttf"
    REQUIRES esp_driver_twai nvs_flash fatfs sdmmc esp_driver_sdmmc esp_timer bt driver)

# The ESP32-P4 has one SDMMC controller shared by both slots: the SD card on
# slot 0 and the ESP32-C6 (Wi-Fi/BLE, via esp_hosted) on slot 1. IDF 6.0's
# sdmmc_host_init() creates a controller and stores it in a file-static, with
# no guard against a second call, so whichever of the two initialises second
# gets ESP_ERR_NOT_FOUND ("no available sd host controller") and its slot never
# comes up. esp_hosted runs first from a constructor, so the card always loses.
#
# Wrapping makes the second call a no-op instead of an error, which is what the
# shared-controller design already assumes. See sd_host_init_shim.c.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=sdmmc_host_init")
