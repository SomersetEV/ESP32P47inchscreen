idf_component_register(
    SRCS "main.c" "can_decode.c" "vehicle_state.c" "rtc_time.c"
         "ui/ui.c" "ui/ui_splash.c" "ui/ui_dash.c" "ui/sim_can.c"
    INCLUDE_DIRS "." "ui"
    EMBED_FILES "assets/logo.png" "assets/Montserrat-Medium.ttf"
    REQUIRES esp_driver_twai nvs_flash fatfs sdmmc esp_driver_sdmmc esp_timer bt driver)
