/*
 * sd_host_init_shim.c — let the SD card and the ESP32-C6 share the one SDMMC
 * controller.
 *
 * The P4 has a single SDMMC controller driving two slots. This board uses both:
 * slot 0 for the microSD card, slot 1 for the ESP32-C6 that provides Wi-Fi and
 * BLE over esp_hosted. IDF 6.0's legacy sdmmc_host_init() does:
 *
 *     sd_host_create_sdmmc_controller(&cfg, &s_ctlr);
 *
 * unconditionally, with s_ctlr a file-static and no check for an existing
 * controller. The claim function hands out the single controller once and
 * returns ESP_ERR_NOT_FOUND after that, so the *second* caller fails with
 * "no available sd host controller" and its slot never comes up.
 *
 * esp_hosted calls it first, from a __attribute__((constructor)) that runs
 * before app_main, so the SD card is always the loser. Nothing in the SD path
 * can reorder around that.
 *
 * The rest of the driver is already written for a shared controller — the same
 * static handle serves both s_slot0 and s_slot1, and per-slot init is a
 * separate call — so the only thing wrong is the missing guard. This wrap adds
 * it: the first call goes through to the real implementation, and later calls
 * succeed without trying to create a second controller.
 *
 * Done with --wrap rather than by patching managed_components/ so the fix
 * survives a component update and stays visible in this project's own source.
 */

#include "esp_err.h"
#include "esp_log.h"

esp_err_t __real_sdmmc_host_init(void);

static const char *TAG = "sdhost_shim";
static bool s_initialised;

esp_err_t __wrap_sdmmc_host_init(void)
{
    if (s_initialised) {
        // Already up for the other slot. The controller is shared by design,
        // so this is success, not a conflict.
        return ESP_OK;
    }

    esp_err_t err = __real_sdmmc_host_init();
    if (err == ESP_OK) {
        s_initialised = true;
    } else {
        ESP_LOGE(TAG, "sdmmc_host_init failed: %s", esp_err_to_name(err));
    }
    return err;
}
