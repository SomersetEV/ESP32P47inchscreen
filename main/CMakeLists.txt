idf_component_register(
    SRCS "main.c" "can_decode.c" "vehicle_state.c" "rtc_time.c" "solar.c"
         "can_handler.c" "can_logger.c" "sd_store.c" "sd_host_init_shim.c"
         "ble_nus.c"
         "ui/ui.c" "ui/ui_splash.c" "ui/ui_dash.c" "ui/sim_can.c"
         "ui/ui_brightness.c"
    INCLUDE_DIRS "." "ui"
    EMBED_FILES "assets/splash_logo.png" "assets/dash_logo.png" "assets/Montserrat-Medium.ttf"
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
#
# The teardown side of the same assumption is worse: a failed SD mount unwinds
# through sdmmc_host_deinit_slot(0), which tries to delete the shared
# controller out from under the C6 and leaves sd_host_isr() dereferencing freed
# state — a load-access-fault panic that looks like "BLE resets the board".
# Slot 0 is never the controller's last user, so its teardown is suppressed.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=sdmmc_host_init"
    "-Wl,--wrap=sdmmc_host_deinit_slot"
    "-Wl,--wrap=sdmmc_host_deinit")
