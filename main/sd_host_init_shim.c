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
 *
 * ── Teardown ────────────────────────────────────────────────────────────────
 *
 * The same single-owner assumption breaks on the way back out, and that one
 * panics rather than just failing a mount.
 *
 * SDMMC_HOST_DEFAULT() sets SDMMC_HOST_FLAG_DEINIT_ARG and deinit_p =
 * sdmmc_host_deinit_slot, so every failed esp_vfs_fat_sdmmc_mount() unwinds
 * through sdmmc_host_deinit_slot(0). That function does:
 *
 *     sd_host_remove_slot(s_slot0);      // fine, slot 0 is ours
 *     sd_host_del_controller(s_ctlr);    // NOT fine, the controller is shared
 *
 * and then deliberately swallows the controller delete's ESP_ERR_INVALID_STATE
 * as ESP_OK "for backward compatibility". Upstream treats that as benign
 * because it assumes whoever tears down a slot owns the controller. Here the
 * refusal is real: it logs
 *
 *     SD_HOST: sd_host_del_sdmmc_controller(353): host controller with slot registered
 *
 * because slot 1 — the ESP32-C6 carrying HCI for BLE — is still registered.
 * With the card absent, can_logger_task retries the mount every 5 s, so this
 * runs repeatedly against the live controller the C6 depends on. The next SDIO
 * interrupt from the C6 then lands in sd_host_isr() on a dangling pointer:
 *
 *     Guru Meditation Error: Core 0 panic'ed (Load access fault)
 *     MEPC: sd_host_isr ... MTVAL: 0x0000009c
 *
 * It presents as "connecting over BLE resets the board", because scanning or
 * connecting is what generates the SDIO traffic that fires the ISR — but the
 * damage is done by the failed SD mount, and BLE is only the victim.
 *
 * Slot 0 is the only slot this firmware ever tears down, and it is never the
 * last user of the controller: slot 1 is brought up before app_main by
 * esp_hosted's constructor and stays up for the life of the process. So the
 * correct teardown is to remove the slot and leave the controller alone.
 * There is no public entry point for "remove slot without deleting the
 * controller", so this wrap drops the slot-0 teardown entirely and lets
 * sd_store_mount() reuse the slot on the next retry — which is what it already
 * does for the LDO handle, and for the same reason.
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

/*
 * Never tear down the shared controller.
 *
 * Reporting ESP_OK is honest here: the caller's contract is "slot 0 is no
 * longer in use", and it is not — the next sd_store_mount() re-runs
 * sdmmc_host_init_slot(0), which re-adds the slot over the existing handle.
 * Failing instead would only push esp_vfs_fat_sdmmc_mount() down another
 * cleanup path for a mount that has already failed for its own reasons.
 */
esp_err_t __wrap_sdmmc_host_deinit_slot(int slot)
{
    (void)slot;
    return ESP_OK;
}

/*
 * Wrapped for the same reason, and this one is stricter: sdmmc_host_deinit()
 * loops over both slot handles and removes whichever are non-NULL, so a single
 * call would drop the C6's slot 1 as well. Nothing in this firmware calls it —
 * the mount path uses deinit_p above — but it is wrapped so that a future
 * caller, or a component update that switches back to the flagless deinit,
 * cannot take the radio down with it.
 */
esp_err_t __wrap_sdmmc_host_deinit(void)
{
    return ESP_OK;
}
