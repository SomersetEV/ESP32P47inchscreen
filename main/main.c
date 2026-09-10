/*
 * SomersetEV Tractor Dash
 * Hardware: Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (1024x600 MIPI-DSI, GT911)
 * Framework: ESP-IDF v6.0.1
 *
 * A 7" dash for an electric Leyland 255. It decodes the same 500 kbps CAN
 * messages the old Arduino Due + Nextion dash used, logs every frame to SD the
 * way the LilyGo pure CAN logger does, and syncs those logs to the existing
 * Flutter phone app over BLE.
 *
 *   Core 1: CAN RX ISR — stamps and queues frames
 *   Core 0: can_logger_task (SD I/O), LVGL, BLE
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "board.h"
#include "vehicle_state.h"
#include "rtc_time.h"
#include "ui.h"
#include "sim_can.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "SomersetEV Tractor Dash v%s starting...", FW_VERSION);

    // NVS holds the session counter that keeps power cycles from colliding.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
    }

    vehicle_state_init();

    // Display first, so the splash is up within about a second of power-on
    // while the rest of the system starts behind it.
    if (ui_start() != ESP_OK) {
        ESP_LOGE(TAG, "Display failed to start");
    }

    // The BSP has already brought up the shared I2C bus by this point.
    rtc_time_init();

    // No-op unless CONFIG_DASH_SIMULATE_CAN is set.
    sim_can_start();

    ESP_LOGI(TAG, "Boot complete");
}
