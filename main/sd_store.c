/*
 * sd_store.c — SD card mounting and space management
 *
 * The card is the only storage; if it will not mount there is nothing this
 * firmware can usefully do, so can_logger_task retries forever.
 *
 * Retention: the card must never fill up and silently stop the log. Free space
 * is checked at boot and periodically during a session; when it falls below the
 * threshold the oldest canlog files are deleted. The file currently being
 * written is always excluded.
 */

#include "sd_store.h"
#include "board.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>

static const char *TAG = "SDSTORE";

// ── Mount ─────────────────────────────────────────────────────────────────────

/*
 * The LilyGo board wired its card over SPI; this one has it on SDMMC slot 0 in
 * 4-bit mode.
 *
 * This does not use bsp_sdcard_mount(), because that function is not safe to
 * retry: it acquires on-chip LDO channel 4 on every call and never releases it
 * when the mount fails. The second attempt then dies with "can't acquire the
 * channel, already in use", and so does every attempt after it — the retry loop
 * poisons itself and reports a misleading error instead of the real one (a card
 * that is absent, unformatted, or not seated).
 *
 * The mount is otherwise identical to the BSP's, so the LDO handle is created
 * once and kept for the lifetime of the process, and retries reuse it.
 */
static sdmmc_card_t         *s_card = NULL;
static sd_pwr_ctrl_handle_t  s_pwr  = NULL;

bool sd_store_mount(void)
{
    // Slot 0 is powered by on-chip LDO channel 4. Acquired once, never
    // released: this is the one place that owns it.
    if (!s_pwr) {
        sd_pwr_ctrl_ldo_config_t ldo_cfg = { .ldo_chan_id = 4 };
        esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &s_pwr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SD LDO init failed: %s", esp_err_to_name(err));
            return false;
        }
    }

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot            = SDMMC_HOST_SLOT_0;
    host.max_freq_khz    = SDMMC_FREQ_HIGHSPEED;
    host.pwr_ctrl_handle = s_pwr;

    const sdmmc_slot_config_t slot_cfg = {
        // Slot 0 uses IO MUX, so the pins are fixed and not specified here.
        .cd    = SDMMC_SLOT_NO_CD,
        .wp    = SDMMC_SLOT_NO_WP,
        .width = 4,
        .flags = 0,
    };

    const esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,   // never reformat a card in a vehicle
        .max_files              = 5,
        .allocation_unit_size   = 64 * 1024,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_cfg,
                                            &mount_cfg, &s_card);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "SD mount failed: no card responding - "
                          "is a card fitted and pushed fully in?");
        } else {
            ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        }
        return false;
    }

    uint64_t total = 0, freeb = 0;
    if (esp_vfs_fat_info(MOUNT_POINT, &total, &freeb) == ESP_OK) {
        ESP_LOGI(TAG, "SD mounted at %s: %lluMB total, %lluMB free",
                 MOUNT_POINT, total >> 20, freeb >> 20);
    } else {
        ESP_LOGI(TAG, "SD mounted at %s", MOUNT_POINT);
    }
    return true;
}

bool sd_store_remount(void)
{
    if (s_card) {
        // Slot teardown inside the unmount is a no-op via the shim in
        // sd_host_init_shim.c, so this cannot take the C6's slot 1 down.
        esp_err_t err = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Unmount failed: %s", esp_err_to_name(err));
        }
        s_card = NULL;
    }
    return sd_store_mount();
}

// ── Free space ────────────────────────────────────────────────────────────────

static bool fat_info(uint64_t *total, uint64_t *freeb)
{
    return esp_vfs_fat_info(MOUNT_POINT, total, freeb) == ESP_OK;
}

uint64_t sd_store_free_bytes(void)
{
    uint64_t total = 0, freeb = 0;
    return fat_info(&total, &freeb) ? freeb : 0;
}

uint64_t sd_store_total_bytes(void)
{
    uint64_t total = 0, freeb = 0;
    return fat_info(&total, &freeb) ? total : 0;
}

// ── Retention ─────────────────────────────────────────────────────────────────

/*
 * Parse "canlog_0042.csv" -> 42. Returns false for any other filename, so
 * unrelated files on the card are never candidates for deletion.
 *
 * Note readdir() on FATFS returns short (8.3) names uppercased when the entry
 * has no LFN, so match case-insensitively.
 */
static bool parse_canlog_id(const char *name, uint32_t *out_id)
{
    if (strncasecmp(name, "canlog_", 7) != 0) return false;

    const char *p = name + 7;
    if (*p < '0' || *p > '9') return false;   // need at least one digit

    uint32_t id = 0;
    while (*p >= '0' && *p <= '9') {
        id = id * 10 + (uint32_t)(*p - '0');
        p++;
    }

    if (strcasecmp(p, ".csv") != 0) return false;

    *out_id = id;
    return true;
}

// Find the lowest-numbered canlog file, excluding keep_id. Returns false when
// no candidate remains. Rescanning per deletion avoids holding a file list in
// RAM, and deletions are rare enough that the cost does not matter.
static bool find_oldest(uint32_t keep_id, uint32_t *out_id)
{
    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed", MOUNT_POINT);
        return false;
    }

    bool     found  = false;
    uint32_t lowest = 0;

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        uint32_t id;
        if (!parse_canlog_id(ent->d_name, &id)) continue;
        if (id == keep_id) continue;
        if (!found || id < lowest) {
            lowest = id;
            found  = true;
        }
    }

    closedir(dir);
    if (found) *out_id = lowest;
    return found;
}

int sd_store_reclaim(uint64_t want_free, uint32_t keep_id)
{
    int deleted = 0;

    while (sd_store_free_bytes() < want_free) {
        uint32_t id;
        if (!find_oldest(keep_id, &id)) {
            ESP_LOGW(TAG, "Low space but no deletable canlog files remain");
            break;
        }

        // A session is a pair of files, so both go together. find_oldest keys
        // off canlog_, which is what makes the pair reclaimable at all: if the
        // canlog deletion fails we stop, otherwise a missing snap is not fatal
        // (a session that ended before its first snapshot has none).
        char raw[64], snap[64];
        snprintf(raw,  sizeof(raw),  MOUNT_POINT "/canlog_%04lu.csv",
                 (unsigned long)id);
        snprintf(snap, sizeof(snap), MOUNT_POINT "/snap_%04lu.csv",
                 (unsigned long)id);

        if (unlink(raw) != 0) {
            ESP_LOGE(TAG, "Failed to delete %s - stopping reclaim", raw);
            break;   // avoid spinning forever on an undeletable file
        }
        unlink(snap);

        ESP_LOGW(TAG, "Reclaimed session %04lu", (unsigned long)id);
        deleted++;
    }

    return deleted;
}
