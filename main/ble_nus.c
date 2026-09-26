/*
 * ble_nus.c — Full NimBLE NUS implementation
 * SomersetEV Tractor Telematics
 *
 * Architecture:
 *   - nimble_port_freertos_init() creates the NimBLE host task internally
 *   - ble_nus_task() does setup, then becomes the command processor loop
 *   - GATT RX callback posts commands to cmd_queue (non-blocking, runs in NimBLE context)
 *   - Command processor handles LIST/GET/DONE/TIME, does SD I/O, sends notifications
 *
 * Android notes:
 *   - Android auto-requests MTU 517 on connect — we honour whatever is negotiated
 *   - Chunk size is set dynamically from negotiated MTU
 *   - A 20ms delay between notification chunks prevents Android BLE stack overflow
 *   - Android requires CCCD subscription before notifications work — standard behaviour,
 *     handled automatically by most Android BLE libraries
 */

#include "ble_nus.h"
#include "vehicle_state.h"
#include "can_logger.h"
#include "can_handler.h"
#include "rtc_time.h"
#include "board.h"
#include "ui.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

static const char *TAG        = "BLE";
#define DEVICE_NAME             "SomersetEV-Tractor"
// MOUNT_POINT comes from board.h, which follows the BSP's own mount point.
#define NVS_NAMESPACE           "telematics"
#define NVS_KEY_LAST_SYNCED     "last_synced"
#define CMD_QUEUE_DEPTH         8
#define CMD_MAX_LEN             32
// Candidates LIST considers; more than one 512-byte reply can carry.
#define LIST_MAX_SESSIONS       64

// ── BLE chunk sizing ─────────────────────────────────────────────────────────
// Default 20 bytes (MTU 23) until Android negotiates MTU up on connect
#define DEFAULT_CHUNK_SIZE      20
#define MAX_CHUNK_SIZE          509     // MTU 512 - 3 bytes ATT overhead

// ── NUS Service and Characteristic UUIDs ────────────────────────────────────
// 128-bit UUIDs stored little-endian as required by NimBLE
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
);
// RX: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (phone writes commands here)
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
);
// TX: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (ESP32 notifies phone)
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
);

static int gap_event_handler(struct ble_gap_event *event, void *arg);

// ── State ────────────────────────────────────────────────────────────────────
static uint16_t      nus_tx_handle  = 0;
static uint16_t      conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static volatile uint16_t negotiated_mtu = DEFAULT_CHUNK_SIZE + 3;
static QueueHandle_t cmd_queue      = NULL;

QueueHandle_t       g_ble_live_queue            = NULL;
volatile app_mode_t g_app_mode                  = APP_MODE_DETECTING;
static volatile uint32_t s_connect_time_ms      = 0;

typedef struct {
    char     text[CMD_MAX_LEN];
    uint16_t len;
} ble_cmd_t;

// ── Notification helper ──────────────────────────────────────────────────────

// How long a notification may wait for a free mbuf before it counts as failed.
#define NOTIFY_RETRY_MS         10
#define NOTIFY_RETRY_LIMIT      200     // 2 s in all

/*
 * Out of mbufs is not a failure, only back-pressure: the pool refills as the
 * controller gets packets onto the air. Treating it as fatal aborted a GET part
 * way with neither END nor ERR, and the phone could only find out by timing out.
 */
static int nus_notify(const void *data, uint16_t len)
{
    for (int attempt = 0; ; attempt++) {
        if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return -1;

        // ble_gatts_notify_custom() consumes the mbuf even when it fails, so
        // every attempt builds a fresh one.
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
        int rc = om ? ble_gatts_notify_custom(conn_handle, nus_tx_handle, om)
                    : BLE_HS_ENOMEM;
        if (rc == 0) return 0;

        if (rc != BLE_HS_ENOMEM || attempt >= NOTIFY_RETRY_LIMIT) {
            ESP_LOGW(TAG, "notify failed: %d", rc);
            return rc;
        }
        vTaskDelay(pdMS_TO_TICKS(NOTIFY_RETRY_MS));
    }
}

// Payload bytes one notification can carry on the current link.
static uint16_t notify_chunk_size(void)
{
    uint16_t chunk = (negotiated_mtu > 3) ? (negotiated_mtu - 3) : DEFAULT_CHUNK_SIZE;
    return (chunk > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : chunk;
}

/*
 * NimBLE silently cuts a notification down to MTU - 3 bytes
 * (ble_att_truncate_to_mtu), so a long reply sent whole lost its tail and,
 * with it, the '\n' the phone waits for. The default ATT MTU here is 256 and
 * iOS offers 185, so a LIST of more than ~20 sessions never arrived intact and
 * sync failed on every connect from then on. Split it instead; the phone
 * reassembles lines across notifications.
 */
static int nus_notify_str(const char *str)
{
    size_t   len   = strlen(str);
    uint16_t chunk = notify_chunk_size();

    while (len > 0) {
        uint16_t n = (len > chunk) ? chunk : (uint16_t)len;
        int rc = nus_notify(str, n);
        if (rc != 0) return rc;
        str += n;
        len -= n;
    }
    return 0;
}

// ── Command handlers ─────────────────────────────────────────────────────────

static void handle_list_command(void)
{
    /*
     * Enumerate completed session files on SD card.
     * The currently-open session (still being written) is excluded.
     *
     * Response: "LIST 0001,3600;0002,1800;\n"
     * Record count is approximate (file_size / 100 bytes per CSV row).
     *
     * Sessions are listed lowest id first, and only the lowest ones when they
     * do not all fit. DONE advances a high-water mark, so the phone must fetch
     * them in ascending order: in readdir order, a DONE for a higher id hid
     * every lower one it had not fetched yet — including any that did not fit
     * in the reply — and those sessions never reached the phone.
     */
    uint32_t last_synced = 0;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, NVS_KEY_LAST_SYNCED, &last_synced);
        nvs_close(nvs);
    }
    // Straight from the logger rather than NVS "session_id" - 1, which named
    // the wrong session once the counter wrapped from 9999 to 1.
    uint32_t active_session = can_logger_session_id();

    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) {
        nus_notify_str("ERR no_sd\n");
        return;
    }

    // The lowest LIST_MAX_SESSIONS unsynced ids, kept sorted ascending.
    uint32_t ids[LIST_MAX_SESSIONS];
    size_t   n_ids = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        uint32_t sid;
        if (sscanf(entry->d_name, "snap_%04lu.csv", &sid) != 1) continue;
        if (sid == active_session) continue;    // skip currently-open session
        if (sid <= last_synced)    continue;    // skip already-synced sessions

        size_t pos = n_ids;
        while (pos > 0 && ids[pos - 1] > sid) pos--;
        if (pos >= LIST_MAX_SESSIONS) continue;  // higher than every kept id
        if (n_ids < LIST_MAX_SESSIONS) n_ids++;  // else the highest drops off
        memmove(&ids[pos + 1], &ids[pos], (n_ids - 1 - pos) * sizeof(ids[0]));
        ids[pos] = sid;
    }
    closedir(dir);

    char response[512] = "LIST ";
    for (size_t i = 0; i < n_ids; i++) {
        char path[64];
        snprintf(path, sizeof(path), MOUNT_POINT "/snap_%04lu.csv", ids[i]);
        struct stat st;
        if (stat(path, &st) != 0 || st.st_size == 0) continue;  // skip empty/unreadable files
        uint32_t records = (uint32_t)(st.st_size / 100);

        char entry_str[32];
        int entry_len = snprintf(entry_str, sizeof(entry_str), "%04lu,%lu;", ids[i], records);
        if (entry_len <= 0) continue;
        // Reserve 2 bytes for the trailing "\n\0"; stop at the first entry that
        // won't fit whole, so the ids sent stay the lowest ones.
        size_t used = strlen(response);
        if (used + (size_t)entry_len + 2 > sizeof(response)) break;
        // memcpy rather than strncat: the bound is already checked above, and
        // GCC 15 cannot see that through strncat's length argument.
        memcpy(response + used, entry_str, (size_t)entry_len + 1);
    }
    size_t used = strlen(response);
    if (used + 2 <= sizeof(response)) {
        response[used]     = '\n';
        response[used + 1] = '\0';
    }
    nus_notify_str(response);
    ESP_LOGI(TAG, "LIST sent: %s", response);
}

static void handle_get_command(uint32_t session_id)
{
    /*
     * Stream a completed session CSV to the phone.
     *
     * Protocol:
     *   1. "DATA <id> <filesize>\n"   — header, phone allocates buffer
     *   2. Raw CSV bytes in chunks    — chunk size = negotiated_mtu - 3
     *   3. "END <id>\n"               — transfer complete
     *
     * 20ms inter-chunk delay prevents Android BLE stack dropping packets.
     * At MTU 512 (509 byte chunks) this gives ~25KB/s — fine for our data rates.
     * A 1-hour session at 1Hz / ~100 bytes/row = ~360KB ≈ 15 seconds transfer time.
     */
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/snap_%04lu.csv", session_id);

    struct stat st;
    if (stat(path, &st) != 0) {
        char err[32];
        snprintf(err, sizeof(err), "ERR not_found %04lu\n", session_id);
        nus_notify_str(err);
        return;
    }

    FILE *f = fopen(path, "r");
    if (!f) {
        nus_notify_str("ERR open_failed\n");
        return;
    }

    // Send header — give Android 50ms to process before streaming begins
    char header[64];
    snprintf(header, sizeof(header), "DATA %04lu %ld\n", session_id, (long)st.st_size);
    if (nus_notify_str(header) != 0) {
        ESP_LOGW(TAG, "GET %04lu: DATA header send failed, aborting", session_id);
        fclose(f);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    uint16_t chunk_size = notify_chunk_size();

    uint8_t  buf[MAX_CHUNK_SIZE];
    size_t   bytes_read;
    uint32_t total_sent = 0;
    uint8_t  last_byte  = '\n';

    while ((bytes_read = fread(buf, 1, chunk_size, f)) > 0) {
        if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGW(TAG, "Disconnected mid-transfer, aborting GET %04lu", session_id);
            fclose(f);
            return;     // Phone discards incomplete session and re-requests on reconnect
        }
        if (nus_notify(buf, (uint16_t)bytes_read) != 0) {
            ESP_LOGW(TAG, "GET %04lu: notify failed mid-transfer, aborting", session_id);
            fclose(f);
            return;
        }
        total_sent += bytes_read;
        last_byte   = buf[bytes_read - 1];
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    fclose(f);

    /*
     * The phone finds the end of the file by a line starting "END". A file
     * whose last row is unterminated (a write cut short by power loss, or a
     * card fault) would glue END onto that row, the phone would never see it,
     * and the session would fail by timeout on every sync. An empty line is
     * skipped by the phone, so terminating here is always safe.
     */
    if (last_byte != '\n') nus_notify_str("\n");

    char end_marker[32];
    snprintf(end_marker, sizeof(end_marker), "END %04lu\n", session_id);
    nus_notify_str(end_marker);

    ESP_LOGI(TAG, "GET %04lu complete — %lu bytes sent", session_id, total_sent);
}

static void handle_done_command(uint32_t session_id)
{
    /*
     * Phone confirms successful receipt and local storage of session.
     * Update NVS last_synced. Never delete from SD — SD is the master archive.
     */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        uint32_t last = 0;
        nvs_get_u32(nvs, NVS_KEY_LAST_SYNCED, &last);
        if (session_id > last) {
            nvs_set_u32(nvs, NVS_KEY_LAST_SYNCED, session_id);
            nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    nus_notify_str("OK\n");
    ESP_LOGI(TAG, "DONE %04lu acknowledged", session_id);
}

static void handle_time_command(uint32_t unix_epoch)
{
    /*
     * Phone sends current Unix timestamp on connect.
     *
     * The telematics board kept (unix_epoch - tick_ms/1000) in
     * vehicle_state as a soft RTC offset. This board sets the system clock
     * and writes the RTC instead, so the time survives a power cycle and the
     * logs carry real dates without the phone. The offset is no longer
     * stored: vehicle_state has one writer (the logger task), and nothing
     * written to SD or BLE reads that field.
     */
    uint32_t current_tick_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    int32_t  offset = (int32_t)unix_epoch - (int32_t)(current_tick_ms / 1000);

    rtc_time_set((time_t)unix_epoch);

    nus_notify_str("OK\n");
    ESP_LOGI(TAG, "TIME synced: unix=%lu tick=%lu offset=%ld",
             unix_epoch, (unsigned long)current_tick_ms, (long)offset);
}

static void handle_trip_marker(trip_marker_t type)
{
    /*
     * Hand the marker to the logger task so it lands in the CSV in the right
     * sequence relative to the data rows. The merged logger takes markers on
     * its own queue rather than sharing the frame queue.
     */
    if (!can_logger_post_marker(type)) {
        ESP_LOGW(TAG, "Marker queue full - trip marker dropped");
        nus_notify_str("ERR queue_full\n");
        return;
    }

    /*
     * Wait for the logger to act on the marker before replying. For TRIP_END
     * that means the files are closed and the next session is open, so the
     * phone can sync the finished session immediately. For TRIP_START it means
     * the marker really reached a log file; with no card there is nothing to
     * start, and replying OK would tell the phone a trip is recording.
     */
    /*
     * TRIP_END can take a while: it clears the saved trip, fsyncs and closes
     * both files, claims a number and may reclaim space before opening the
     * next pair. Replying early made the phone sync before the finished
     * session was listable. Each limit sits inside the phone's own wait for
     * the reply (5 s for TRIP_START, 10 s for TRIP_END).
     */
    const uint32_t wait_ms = (type == TRIP_MARKER_END) ? 8000 : 4000;
    bool ok = false;
    if (!can_logger_wait_marker(wait_ms, &ok)) {
        ESP_LOGW(TAG, "Logger did not handle the trip marker in time");
    } else if (!ok) {
        nus_notify_str("ERR no_log\n");
        ESP_LOGW(TAG, "Trip marker failed - no log file open");
        return;
    }
    nus_notify_str("OK\n");
    ESP_LOGI(TAG, "Trip marker queued: %s",
             type == TRIP_MARKER_START ? "TRIP_START" : "TRIP_END");
}

static void handle_status_command(void)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "STATUS trip=%d\n",
             can_logger_trip_active() ? 1 : 0);
    nus_notify_str(buf);
}

static void handle_wifi_mode_command(void)
{
    /*
     * The telematics board rebooted into a Wi-Fi web interface here. This dash
     * has no web interface yet, and the command is answered rather than ignored
     * so the app gets a definite reply instead of timing out.
     */
    nus_notify_str("ERR unsupported\n");
    ESP_LOGI(TAG, "WIFI_MODE requested but not supported on this build");
}

static void handle_summary_command(uint32_t session_id)
{
    /*
     * Read the snap_XXXX.csv file and find the TRIP_END summary row.
     * Parse its fields and emit a single compact JOB line to the phone.
     *
     * TRIP_END row format (written by sd_logger):
     *   TRIP_END,<duration_s>,<ah>,<kwh>,<soc_start>,<soc_end>,<peak_a>,
     *            <start_unix>,<peak_rpm>,<peak_motor_c10>,<peak_inv_c10>,<peak_bms_c10>,,
     *
     * JOB response format:
     *   JOB <id>,<start_unix>,<duration_s>,<ah>,<kwh>,<soc_start>,<soc_end>,
     *       <peak_a>,<peak_rpm>,<peak_motor_c>,<peak_inv_c>,<peak_bms_c>\n
     */
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/snap_%04lu.csv", session_id);

    FILE *f = fopen(path, "r");
    if (!f) {
        char err[40];
        snprintf(err, sizeof(err), "ERR not_found %04lu\n", session_id);
        nus_notify_str(err);
        return;
    }

    // Scan the whole file — keep the last TRIP_END line found
    char line[256];
    char trip_end_line[256] = {0};
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "TRIP_END", 8) == 0) {
            // Both buffers are the same size and line is NUL-terminated by
            // fgets, so this cannot truncate; snprintf says so in a way the
            // compiler can check.
            snprintf(trip_end_line, sizeof(trip_end_line), "%s", line);
        }
    }
    fclose(f);

    if (trip_end_line[0] == '\0') {
        char err[40];
        snprintf(err, sizeof(err), "ERR no_trip %04lu\n", session_id);
        nus_notify_str(err);
        return;
    }

    // Parse TRIP_END fields (gracefully handle older files missing the new columns)
    unsigned long duration_s = 0, start_unix = 0;
    float   ah = 0.0f, kwh = 0.0f, peak_a = 0.0f;
    unsigned soc_start = 0, soc_end = 0;
    int peak_rpm = 0, peak_motor_c10 = 0, peak_inv_c10 = 0, peak_bms_c10 = 0;

    int n = sscanf(trip_end_line,
        "TRIP_END,%lu,%f,%f,%u,%u,%f,%lu,%d,%d,%d,%d",
        &duration_s, &ah, &kwh, &soc_start, &soc_end, &peak_a,
        &start_unix, &peak_rpm, &peak_motor_c10, &peak_inv_c10, &peak_bms_c10);
    (void)n;  // fields 8-11 silently default to 0 if absent (old firmware files)

    char job_line[192];
    snprintf(job_line, sizeof(job_line),
        "JOB %lu,%lu,%lu,%.2f,%.3f,%u,%u,%.1f,%d,%.1f,%.1f,%.1f\n",
        (unsigned long)session_id, start_unix, duration_s,
        ah, kwh, soc_start, soc_end, peak_a,
        peak_rpm,
        peak_motor_c10 / 10.0f,
        peak_inv_c10   / 10.0f,
        peak_bms_c10   / 10.0f);
    nus_notify_str(job_line);
    ESP_LOGI(TAG, "SUMMARY %04lu: %s", session_id, job_line);
}

static void dispatch_command(const char *cmd, uint16_t len)
{
    if (g_app_mode != APP_MODE_WEB_INTERFACE) {
        g_app_mode = APP_MODE_TELEMATICS;
    }
    uint32_t arg;
    if      (strncmp(cmd, "LIST", 4) == 0)              { handle_list_command();                    }
    else if (sscanf(cmd, "SUMMARY %lu", &arg) == 1)    { handle_summary_command(arg);              }
    else if (sscanf(cmd, "GET %lu",     &arg) == 1)    { handle_get_command(arg);                  }
    else if (sscanf(cmd, "DONE %lu",    &arg) == 1)    { handle_done_command(arg);                 }
    else if (sscanf(cmd, "TIME %lu",    &arg) == 1)    { handle_time_command(arg);                 }
    else if (strncmp(cmd, "TRIP_START", 10) == 0)      { handle_trip_marker(TRIP_MARKER_START);    }
    else if (strncmp(cmd, "TRIP_END",   8) == 0)       { handle_trip_marker(TRIP_MARKER_END);      }
    else if (strncmp(cmd, "STATUS",     6) == 0)       { handle_status_command();                  }
    else if (strncmp(cmd, "WIFI_MODE",  9) == 0)       { handle_wifi_mode_command();                }
    else {
        ESP_LOGW(TAG, "Unknown cmd: %.*s", len, cmd);
        nus_notify_str("ERR unknown_cmd\n");
    }
}

// ── GATT RX callback ─────────────────────────────────────────────────────────

static int nus_rx_access_cb(uint16_t conn_hdl, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_hdl; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    // This runs in NimBLE host task context — just copy data and post to queue
    ble_cmd_t cmd = {0};
    uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
    if (om_len >= CMD_MAX_LEN) om_len = CMD_MAX_LEN - 1;
    ble_hs_mbuf_to_flat(ctxt->om, cmd.text, om_len, &cmd.len);
    cmd.text[cmd.len] = '\0';

    // Strip trailing CR/LF
    while (cmd.len > 0 &&
           (cmd.text[cmd.len-1] == '\n' || cmd.text[cmd.len-1] == '\r')) {
        cmd.text[--cmd.len] = '\0';
    }

    if (xQueueSend(cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — dropped: %s", cmd.text);
    }
    return 0;
}

static int nus_tx_access_cb(uint16_t conn_hdl, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_hdl; (void)attr_handle; (void)ctxt; (void)arg;
    return 0;
}

// ── GATT service table ───────────────────────────────────────────────────────

static const struct ble_gatt_svc_def nus_gatt_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = &nus_rx_uuid.u,
                .access_cb = nus_rx_access_cb,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid       = &nus_tx_uuid.u,
                .access_cb  = nus_tx_access_cb,
                .val_handle = &nus_tx_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        }
    },
    { 0 }
};

// ── GAP event handler ────────────────────────────────────────────────────────

static void restart_advertising(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_handler, NULL);
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle       = event->connect.conn_handle;
            negotiated_mtu    = DEFAULT_CHUNK_SIZE + 3;
            // In web-interface mode there's no Telematics/Speedo detection —
            // stay in APP_MODE_WEB_INTERFACE so command dispatch keeps working.
            if (g_app_mode != APP_MODE_WEB_INTERFACE) {
                g_app_mode = APP_MODE_DETECTING;
            }
            s_connect_time_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
            ui_dash_set_ble(true);
            ESP_LOGI(TAG, "Phone connected — handle=%d", conn_handle);
        } else {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            restart_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle       = BLE_HS_CONN_HANDLE_NONE;
        // Web-interface mode persists across BLE disconnect — the user may now
        // be driving the web UI from a browser instead of the phone app.
        if (g_app_mode != APP_MODE_WEB_INTERFACE) {
            g_app_mode = APP_MODE_DETECTING;
        }
        s_connect_time_ms = 0;
        if (g_ble_live_queue) {
            raw_can_log_t tmp;
            while (xQueueReceive(g_ble_live_queue, &tmp, 0) == pdTRUE) {}
        }
        ui_dash_set_ble(false);
        ESP_LOGI(TAG, "Disconnected — reason=%d", event->disconnect.reason);
        restart_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        /*
         * Android requests MTU 517 on connect. ESP32 NimBLE default max is 512.
         * Actual agreed value ends up here. Usable payload = mtu - 3.
         */
        negotiated_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU=%d, chunk size=%d bytes", negotiated_mtu, negotiated_mtu - 3);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "TX notifications %s",
                 event->subscribe.cur_notify ? "enabled" : "disabled");
        break;

    default:
        break;
    }
    return 0;
}

// ── NimBLE host task ─────────────────────────────────────────────────────────

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();          // blocks until nimble_port_stop()
    nimble_port_freertos_deinit();
}

static void on_ble_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "No BT address");
        return;
    }

    // Advertising data: flags + name only (23 bytes — fits in 31-byte limit)
    // UUID128 goes in the scan response so it doesn't push us over the limit
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags            = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name             = (uint8_t *)DEVICE_NAME;
    adv_fields.name_len         = strlen(DEVICE_NAME);
    adv_fields.name_is_complete = 1;
    int rc2 = ble_gap_adv_set_fields(&adv_fields);
    if (rc2 != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed: %d", rc2);
        return;
    }

    // Scan response: NUS UUID128 (visible on active scan)
    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.uuids128             = &nus_svc_uuid;
    rsp_fields.num_uuids128         = 1;
    rsp_fields.uuids128_is_complete = 1;
    rc2 = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc2 != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields failed: %d", rc2);
        return;
    }

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_handler, NULL);

    ESP_LOGI(TAG, "Advertising as \"%s\"", DEVICE_NAME);
}

static void on_ble_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset — reason=%d", reason);
}

// ── Public entry point ───────────────────────────────────────────────────────

void ble_nus_task(void *pvParameters)
{
    // Trip markers now go to the logger through can_logger_post_marker(), so
    // there is no shared queue to receive here.
    (void)pvParameters;
    ESP_LOGI(TAG, "BLE NUS task starting");

    cmd_queue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(ble_cmd_t));
    configASSERT(cmd_queue);
    g_ble_live_queue = xQueueCreate(128, sizeof(raw_can_log_t));
    configASSERT(g_ble_live_queue);

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);

    int rc = ble_gatts_count_cfg(nus_gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(nus_gatt_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb  = on_ble_sync;
    ble_hs_cfg.reset_cb = on_ble_reset;

    // NimBLE host runs in its own task — this task becomes the command processor
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "Command processor running");
    ble_cmd_t cmd;
    while (1) {
        TickType_t timeout = (g_app_mode == APP_MODE_SPEEDO)
                             ? pdMS_TO_TICKS(100)
                             : pdMS_TO_TICKS(1000);
        if (xQueueReceive(cmd_queue, &cmd, timeout) == pdTRUE) {
            ESP_LOGI(TAG, "RX: [%s]", cmd.text);
            dispatch_command(cmd.text, cmd.len);
        } else if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            if (g_app_mode == APP_MODE_DETECTING) {
                uint32_t now_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
                if (s_connect_time_ms > 0 && (now_ms - s_connect_time_ms) >= 3000) {
                    g_app_mode = APP_MODE_SPEEDO;
                    ESP_LOGI(TAG, "No command received — entering Speedo mode");
                }
            } else if (g_app_mode == APP_MODE_SPEEDO) {
                // Drain live queue and stream CAN frames as GVRET CSV to Speedo app
                raw_can_log_t frame;
                char buf[512];
                int buf_len = 0;
                // A notification carries at most one chunk; more is cut off.
                int buf_cap = notify_chunk_size();
                if (buf_cap > (int)sizeof(buf)) buf_cap = (int)sizeof(buf);
                bool disconnected = false;
                while (!disconnected &&
                       xQueueReceive(g_ble_live_queue, &frame, 0) == pdTRUE) {
                    char line[64];
                    int line_len = snprintf(line, sizeof(line),
                        "%lu,%lu,0,0,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                        (unsigned long)frame.tick_ms,
                        (unsigned long)frame.id,
                        frame.dlc,
                        frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
                    if (line_len <= 0 || line_len >= (int)sizeof(line)) continue;
                    if (buf_len > 0 && buf_len + line_len > buf_cap) {
                        if (nus_notify(buf, (uint16_t)buf_len) != 0) {
                            disconnected = true;
                            break;
                        }
                        vTaskDelay(pdMS_TO_TICKS(20));
                        buf_len = 0;
                    }
                    memcpy(buf + buf_len, line, line_len);
                    buf_len += line_len;
                }
                if (!disconnected && buf_len > 0) nus_notify(buf, (uint16_t)buf_len);
            } else if (g_app_mode == APP_MODE_TELEMATICS) {
                // Telematics mode: periodic CAN activity heartbeat
                char buf[32];
                snprintf(buf, sizeof(buf), "CAN %lu\n", can_handler_frame_count());
                nus_notify_str(buf);
            }
            // APP_MODE_WEB_INTERFACE: no periodic heartbeat — command/reply only
        }
    }
}
