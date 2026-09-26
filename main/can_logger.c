/*
 * can_logger.c — SD card CAN log writer
 *
 * Architecture:
 *   The CAN ISR (Core 1) drops frames into s_frame_queue. This task (Core 0)
 *   is the only thing that touches the filesystem, so no blocking SD write
 *   ever runs in interrupt context or stalls the capture path.
 *
 * Power loss:
 *   Power is cut without warning, so nothing may depend on an orderly
 *   shutdown. Two consequences shape this file:
 *
 *   1. The file number is incremented and committed to NVS *before* the log is
 *      opened, so a cut at any instant still leaves the next boot a fresh,
 *      unused number. Incrementing at close would mean every boot reused the
 *      same name and overwrote the previous session.
 *
 *   2. Data is flushed and fsync'd on a fixed ~1 s cadence. fflush() alone
 *      only pushes libc's buffer into FATFS; fsync() is what commits the data
 *      sectors *and* the directory entry, so the file has a correct size and
 *      is readable after a cut. Worst-case loss is the last flush interval.
 *
 * Retention:
 *   Free space is checked at boot and every 60 s. Below the threshold the
 *   oldest logs are deleted so the card can never fill and silently end the
 *   recording. A session is a pair of files and both go together.
 *
 * Two files per session:
 *   canlog_NNNN.csv is the raw capture, every frame verbatim.
 *   snap_NNNN.csv is a 1 Hz snapshot of decoded values in the SNAP1 layout the
 *   phone app already parses, plus trip markers and an end-of-trip summary.
 *   Both carry the same session number.
 */

#include "can_logger.h"
#include "can_handler.h"
#include "can_decode.h"
#include "sd_store.h"
#include "vehicle_state.h"
#include "rtc_time.h"
#include "ble_nus.h"
#include "board.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "CANLOG";

// Namespace and key match the telematics firmware, so a card and a board that
// have been through that build keep counting from where they left off.
#define NVS_NAMESPACE     "telematics"
#define NVS_KEY_CANLOG_ID "session_id"

// 1 Hz snapshot cadence for snap_NNNN.csv.
#define SNAP_INTERVAL_MS  1000

// A saturated 500 kbps bus is ~3800 frames/s. 4096 entries buys ~1 s of
// absorption, at ~96 KB of internal RAM. That covers a slow two-file fsync or
// the unlink of a large old session during reclaim, either of which can block
// the drain for well over the ~130 ms the old 512-entry queue gave.
#define FRAME_QUEUE_DEPTH 4096

// Frames drained per pass before the housekeeping runs again. Bounded so the
// flush cadence holds under sustained load.
#define DRAIN_BATCH_MAX   512

// Durability cadence. Worst-case data loss on a power cut is one interval.
#define FLUSH_INTERVAL_MS 1000

// A multiple of the 512-byte FAT sector, so libc hands FATFS whole sectors.
// 16 KB means a handful of multi-sector writes per second at full bus load
// rather than ~40 small ones. Nothing extra is at risk on a power cut: the
// buffer is flushed and fsync'd by commit() every FLUSH_INTERVAL_MS.
#define FILE_BUF_BYTES 16384

// Retention: keep 10% of the card free, checked once a minute.
#define FREE_SPACE_PCT      10
#define SPACE_CHECK_INTERVAL_MS 60000

// Health line cadence.
#define STATUS_INTERVAL_MS 30000

// How long to keep retrying a failed open before giving up on this file and
// falling through to the status loop, so the fault is at least visible.
#define OPEN_RETRY_LIMIT   12
#define OPEN_RETRY_MS      5000

// Consecutive failed commits before the current file is written off and a fresh
// one is opened. Once libc latches a stream error every later write is silently
// discarded, so persisting with the same FILE* would log into a void.
#define COMMIT_FAIL_LIMIT  3

// How often to re-issue a bus-off recovery request while the node is still off
// the bus. Recovery completes asynchronously, so this is a retry for a request
// the driver refused, not a poll for completion.
#define RECOVER_RETRY_MS   1000

static QueueHandle_t     s_frame_queue = NULL;
static volatile uint32_t s_drops       = 0;   // frames lost to a full queue
static volatile bool     s_bus_off     = false;

static FILE     *s_file    = NULL;
static char     *s_filebuf = NULL;
static uint32_t  s_log_id  = 0;
static uint64_t  s_want_free = 0;
static uint32_t  s_written   = 0;   // frames written to the current file
static uint32_t  s_reported_drops = 0;
static uint32_t  s_commit_fails   = 0;  // consecutive failed commits
static uint32_t  s_reopens        = 0;  // files abandoned after write errors

// ── Snapshot file and trip state ─────────────────────────────────────────────

static FILE            *s_snap = NULL;
static QueueHandle_t    s_marker_queue = NULL;
static SemaphoreHandle_t s_marker_sem  = NULL;   // given when a marker is handled
static volatile bool     s_marker_ok   = false;  // its outcome, set before the give

// Wall-clock minus esp_timer, in microseconds, sampled once per drain pass.
// Adding it to a frame's ISR stamp gives the wall-clock time the frame
// *arrived*, not the time it was written, which lags by the queue latency.
static int64_t s_unix_off_us = 0;
static bool    s_unix_off_ok = false;

static bool     s_trip_active = false;
static uint32_t s_trip_start_tick;
static int32_t  s_trip_start_ah;
static int32_t  s_trip_start_kwh;
static uint8_t  s_trip_start_soc;
static int32_t  s_trip_peak_current;
static int16_t  s_trip_peak_rpm;
static int16_t  s_trip_peak_motor_c;
static int16_t  s_trip_peak_inv_c;
static int16_t  s_trip_peak_bms_c;

uint32_t can_logger_session_id(void) { return s_log_id; }
bool     can_logger_trip_active(void) { return s_trip_active; }
bool     can_logger_is_logging(void)  { return s_file != NULL; }

bool can_logger_post_marker(trip_marker_t type)
{
    if (!s_marker_queue || !s_marker_sem) return false;
    // A wait that timed out leaves its completion behind; clear it so the next
    // wait cannot return early on the previous marker's give.
    xSemaphoreTake(s_marker_sem, 0);
    return xQueueSend(s_marker_queue, &type, pdMS_TO_TICKS(100)) == pdTRUE;
}

bool can_logger_wait_marker(uint32_t timeout_ms, bool *ok)
{
    if (!s_marker_sem) return false;
    if (xSemaphoreTake(s_marker_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return false;
    }
    if (ok) *ok = s_marker_ok;
    return true;
}

static void marker_done(bool ok)
{
    s_marker_ok = ok;
    if (s_marker_sem) xSemaphoreGive(s_marker_sem);
}

// Wall-clock milliseconds, or 0 while the clock is unset. The raw log carries
// this next to the monotonic microsecond stamp so a capture can be tied to a
// real date once the RTC or the phone has set the time.
static uint64_t unix_ms_now(void)
{
    if (!rtc_time_valid()) return 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

// ── ISR entry points ──────────────────────────────────────────────────────────

/*
 * IRAM_ATTR because the TWAI ISR is cache-safe (CONFIG_TWAI_ISR_CACHE_SAFE):
 * it keeps running while a flash write has the cache off, and everything it
 * calls must be reachable then too.
 */
void IRAM_ATTR can_logger_submit_from_isr(const can_frame_t *f, BaseType_t *woken)
{
    if (s_frame_queue == NULL) return;
    if (xQueueSendFromISR(s_frame_queue, f, woken) != pdTRUE) {
        s_drops++;   // queue full: the SD card is not keeping up
    }
}

void IRAM_ATTR can_logger_notify_bus_off_from_isr(void)
{
    s_bus_off = true;
}

void can_logger_submit(const can_frame_t *f)
{
    if (s_frame_queue == NULL) return;
    if (xQueueSend(s_frame_queue, f, 0) != pdTRUE) {
        s_drops++;
    }
}

// ── NVS: the per-power-cycle file counter ─────────────────────────────────────

static bool session_files_exist(uint32_t id)
{
    char path[64];
    struct stat st;

    snprintf(path, sizeof(path), MOUNT_POINT "/canlog_%04lu.csv", (unsigned long)id);
    if (stat(path, &st) == 0) return true;
    snprintf(path, sizeof(path), MOUNT_POINT "/snap_%04lu.csv", (unsigned long)id);
    return stat(path, &st) == 0;
}

/*
 * Claim the next log number. The increment is committed before the caller
 * opens the file, so an unexpected power cut can never hand the next boot a
 * number that is already in use.
 *
 * NVS alone is not enough, because the files are opened with "w". A counter
 * that restarts — a new board, an erased or reinitialised NVS partition, or
 * the wrap from 9999 — would truncate the sessions already on the card under
 * the same names, including ones the phone has not fetched yet. Numbers still
 * in use on the card are skipped instead.
 */
static uint32_t claim_log_id(void)
{
    nvs_handle_t nvs;
    uint32_t     id = 1;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed - falling back to canlog_0001.csv");
        return id;
    }

    nvs_get_u32(nvs, NVS_KEY_CANLOG_ID, &id);
    if (id == 0) id = 1;

    // Bounded, so a card holding every number cannot stall the logger.
    for (uint32_t tries = 1; tries < 9999 && session_files_exist(id); tries++) {
        id = (id >= 9999) ? 1 : id + 1;
    }

    // Wrap well before the %04lu field would overflow into a 5th digit.
    uint32_t next = (id >= 9999) ? 1 : id + 1;
    if (nvs_set_u32(nvs, NVS_KEY_CANLOG_ID, next) == ESP_OK) {
        nvs_commit(nvs);
    } else {
        ESP_LOGE(TAG, "NVS write failed - next boot may reuse this number");
    }
    nvs_close(nvs);

    return id;
}

// ── File handling ─────────────────────────────────────────────────────────────

/*
 * Force everything to the card: libc buffer -> FATFS -> physical media and
 * directory entry. Without the fsync the file would have a stale size after a
 * power cut and could read as empty.
 *
 * Returns false if anything failed. This matters more than it looks: once a
 * write error latches the stream error flag, every subsequent fwrite silently
 * discards data. Without this check the logger would keep counting frames and
 * printing a healthy status line while nothing reached the card at all.
 */
static bool commit(void)
{
    if (!s_file) return false;

    int64_t t0 = esp_timer_get_time();
    bool ok = (fflush(s_file) == 0);
    if (ok) ok = (fsync(fileno(s_file)) == 0);
    if (ferror(s_file)) ok = false;

    /*
     * The snapshot file gets the same treatment on the same cadence, so a power
     * cut costs at most one interval from both files rather than leaving the
     * snapshot hours behind the raw log. Its failure does not condemn the raw
     * log though — that is the record that matters, so the return value still
     * tracks s_file only.
     */
    if (s_snap) {
        if (fflush(s_snap) == 0) {
            fsync(fileno(s_snap));
        }
    }

    int64_t took = esp_timer_get_time() - t0;

    /*
     * The task cannot drain the frame queue while this blocks, so a slow card
     * shows up as dropped frames rather than as an error. Say so plainly —
     * a healthy card commits in a few milliseconds.
     */
    if (took > 100000) {
        ESP_LOGW(TAG, "Slow SD commit: %lldms - card cannot keep up",
                 (long long)(took / 1000));
    }

    return ok;
}

static bool open_log(void)
{
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/canlog_%04lu.csv",
             (unsigned long)s_log_id);

    s_file = fopen(path, "w");
    if (!s_file) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return false;
    }

    if (s_filebuf) setvbuf(s_file, s_filebuf, _IOFBF, FILE_BUF_BYTES);

    fputs("us,unix_ms,id,ext,rtr,dlc,data\n", s_file);

    // Make the file valid on the card immediately, and treat a header that will
    // not commit as a failed open — better to retry than to log into a file
    // that was never really created.
    if (!commit()) {
        ESP_LOGE(TAG, "Could not commit header to %s", path);
        fclose(s_file);
        s_file = NULL;
        return false;
    }

    s_written      = 0;
    s_commit_fails = 0;

    ESP_LOGI(TAG, "Logging to canlog_%04lu.csv", (unsigned long)s_log_id);
    return true;
}

/*
 * The 1 Hz snapshot the phone app reads. The first 14 columns are the
 * telematics SNAP1 layout byte for byte — the app's parser is not being
 * changed — with charger_temp_c10 and unix_time appended. The app already
 * reads p[14], and extra trailing columns are ignored by it.
 *
 * A missing snapshot file is not fatal: the raw capture is the record that
 * matters, and the dash keeps working without it.
 */
static bool open_snap(void)
{
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/snap_%04lu.csv",
             (unsigned long)s_log_id);

    s_snap = fopen(path, "w");
    if (!s_snap) {
        ESP_LOGW(TAG, "Failed to open %s - continuing without snapshots", path);
        return false;
    }

    fputs("SNAP1,tick_ms,soc_pct,pack_v_bms_mv,pack_i_ma,"
          "isa_kw_w,isa_as,motor_rpm,motor_temp_c10,inv_temp_c10,"
          "bms_tmax_c10,bms_tmin_c10,cell_v_max_mv,cell_v_min_mv,"
          "charger_temp_c10,unix_time\n", s_snap);
    fflush(s_snap);
    ESP_LOGI(TAG, "Snapshots to snap_%04lu.csv", (unsigned long)s_log_id);
    return true;
}

static void write_snapshot(uint32_t tick_ms)
{
    if (!s_snap) return;

    vehicle_state_t st;
    vehicle_state_snapshot(&st);
    const log_record_t *r = &st.latest;

    if (s_trip_active) {
        int32_t abs_ma = r->pack_current_ma < 0 ? -r->pack_current_ma
                                                :  r->pack_current_ma;
        if (abs_ma          > s_trip_peak_current) s_trip_peak_current = abs_ma;
        if (r->motor_rpm    > s_trip_peak_rpm)     s_trip_peak_rpm     = r->motor_rpm;
        if (r->motor_temp   > s_trip_peak_motor_c) s_trip_peak_motor_c = r->motor_temp;
        if (r->inverter_temp > s_trip_peak_inv_c)  s_trip_peak_inv_c   = r->inverter_temp;
        if (r->bms_temp_max > s_trip_peak_bms_c)   s_trip_peak_bms_c   = r->bms_temp_max;
    }

    time_t now = rtc_time_valid() ? time(NULL) : 0;

    fprintf(s_snap,
        "SNAP1,%lu,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d,%lu\n",
        (unsigned long)tick_ms,
        (unsigned)r->soc,
        (unsigned)(r->pack_voltage_bms * 10u),   // 0.01 V units -> mV
        (int)r->pack_current_ma,
        (int)r->isa_kw,
        (int)r->isa_ah,
        (int)r->motor_rpm,
        (int)r->motor_temp,
        (int)r->inverter_temp,
        (int)r->bms_temp_max,
        (int)r->bms_temp_min,
        (unsigned)r->cell_voltage_max,
        (unsigned)r->cell_voltage_min,
        (int)r->charger_temp,
        (unsigned long)now);

    // The snapshot file rides the same 1 s commit cadence as the raw log, so
    // it is fsync'd in commit() rather than flushed on a counter here.
}

// Marker rows keep each file's own column count so both still parse.
static void write_trip_start_rows(void)
{
    if (s_file) fprintf(s_file, "TRIP_START,,,,,,\n");
    if (s_snap) fprintf(s_snap, "TRIP_START,,,,,,,,,,,,,,,\n");
    commit();
}

/*
 * The current file has stopped accepting writes. Abandon it and start a new one
 * under a fresh number: the card may still be usable (a bad sector, a transient
 * error), and a new file is the only way to find out. Never reuses the old
 * number, so whatever was recovered from the old file stays intact.
 */
static bool reopen_log(void)
{
    if (s_file) {
        fclose(s_file);   // may itself fail; nothing useful to do if it does
        s_file = NULL;
    }
    // The snapshot belongs to the session being abandoned, so it goes with it
    // rather than trailing rows from the new session into the old file.
    if (s_snap) {
        fclose(s_snap);
        s_snap = NULL;
    }

    s_log_id = claim_log_id();
    sd_store_reclaim(s_want_free, s_log_id);

    if (!open_log()) return false;
    open_snap();

    /*
     * A trip that was running carries on into the new files. Without its own
     * TRIP_START the phone sees the old file's trip end at the cut, and every
     * row after it lands outside any job; the eventual TRIP_END then has no
     * start to pair with and is discarded.
     */
    if (s_trip_active) write_trip_start_rows();

    s_reopens++;
    // Drops are counted per-file; the old file's tally went with it.
    s_reported_drops = s_drops;
    return true;
}

/*
 * Drop the mount and mount again, so a card that was reseated or swapped is
 * actually picked up. Any files still open belong to the old mount and are
 * closed first. The retention target is recomputed because the new card
 * may be a different size.
 */
static bool remount_card(void)
{
    if (s_file) { fclose(s_file); s_file = NULL; }
    if (s_snap) { fclose(s_snap); s_snap = NULL; }

    if (!sd_store_remount()) return false;
    s_want_free = sd_store_total_bytes() * FREE_SPACE_PCT / 100;
    return true;
}

/*
 * Hand-rolled number formatting for the per-frame path. snprintf with 64-bit
 * conversions, called once per field and again per data byte, was the bulk of
 * the logger's CPU time at full bus rate. These produce exactly what the
 * "%lld" / "%llu" / "%0*lX" / "%02X" conversions did.
 */
static const char HEXDIGITS[] = "0123456789ABCDEF";

static char *put_u32(char *p, uint32_t v)
{
    char tmp[10];
    int  n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) *p++ = tmp[--n];
    return p;
}

// One 64-bit divide per call rather than one per digit, which matters on a
// 32-bit core where each is a library call.
static char *put_u64(char *p, uint64_t v)
{
    if (v <= UINT32_MAX) return put_u32(p, (uint32_t)v);

    uint64_t hi = v / 1000000000u;
    uint32_t lo = (uint32_t)(v - hi * 1000000000u);
    p = put_u64(p, hi);
    for (int i = 8; i >= 0; i--) { p[i] = (char)('0' + lo % 10); lo /= 10; }
    return p + 9;
}

// Uppercase hex, zero-padded to at least min_digits, like "%0*lX".
static char *put_hex(char *p, uint32_t v, int min_digits)
{
    int digits = 1;
    for (uint32_t t = v >> 4; t; t >>= 4) digits++;
    if (digits < min_digits) digits = min_digits;
    for (int i = digits - 1; i >= 0; i--) { p[i] = HEXDIGITS[v & 0xF]; v >>= 4; }
    return p + digits;
}

/*
 * Sample the wall-clock offset once per drain pass rather than calling
 * gettimeofday() per frame. The two clocks tick together, so the offset only
 * moves when the time is set (RTC at boot, or the phone's TIME command).
 */
static void refresh_unix_offset(void)
{
    if (!rtc_time_valid()) { s_unix_off_ok = false; return; }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t unix_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
    s_unix_off_us = unix_us - esp_timer_get_time();
    s_unix_off_ok = true;
}

/*
 * Format one frame as CSV into a single buffer and write it with one fwrite.
 *
 * Extended IDs print as 8 hex digits and standard as 3, so the two are
 * distinguishable at a glance as well as via the ext column.
 *
 * unix_ms is derived from the ISR's receive stamp, so it agrees with the us
 * column. It used to be read at write time, which put it behind by however
 * long the frame had sat in the queue.
 */
static void write_frame(const can_frame_t *f)
{
    if (!s_file) return;

    // Worst case: 20 + 1 + 20 + 1 + 10 + 7 + 23 + 1 = 83 chars.
    char  line[128];
    char *p = line;

    uint64_t unix_ms = s_unix_off_ok
                     ? (uint64_t)(f->us + s_unix_off_us) / 1000u
                     : 0;
    uint8_t  dlc = f->dlc <= 8 ? f->dlc : 8;

    p = put_u64(p, (uint64_t)f->us);  *p++ = ',';
    p = put_u64(p, unix_ms);          *p++ = ',';
    *p++ = '0'; *p++ = 'x';
    p = put_hex(p, f->id, f->ext ? 8 : 3);  *p++ = ',';
    p = put_u32(p, f->ext);           *p++ = ',';
    p = put_u32(p, f->rtr);           *p++ = ',';
    p = put_u32(p, dlc);              *p++ = ',';

    for (uint8_t i = 0; i < dlc; i++) {
        if (i) *p++ = ' ';
        *p++ = HEXDIGITS[f->data[i] >> 4];
        *p++ = HEXDIGITS[f->data[i] & 0xF];
    }
    *p++ = '\n';

    fwrite(line, 1, (size_t)(p - line), s_file);
    s_written++;
}

/*
 * Record dropped frames in the log itself, so a gap in the capture is visible
 * to whoever reads the card rather than only in the serial console.
 *
 * Emitted as a well-formed CSV row rather than a "# comment": a bare comment
 * line has none of the six declared columns and makes strict readers (pandas,
 * most CAN tooling) either error or quietly discard rows around it. A row with
 * an empty id and DROPPED=n in the data column parses everywhere and is still
 * obvious to a human.
 */
static void note_drops(void)
{
    uint32_t drops = s_drops;   // sampled once; the ISR may bump it meanwhile
    if (drops == s_reported_drops || !s_file) return;

    // Seven columns now that unix_ms exists, so the row still parses.
    int w = fprintf(s_file, "%lld,%llu,,,,0,DROPPED=%lu\n",
                    (long long)esp_timer_get_time(),
                    (unsigned long long)unix_ms_now(),
                    (unsigned long)(drops - s_reported_drops));

    // Only bank the count if it actually reached the stream, so a failed write
    // does not silently erase the record of a gap.
    if (w > 0) s_reported_drops = drops;
}

// ── Trip markers ──────────────────────────────────────────────────────────────

static void handle_trip_start(void)
{
    /*
     * The phone can lose track of a running trip: its STATUS query after a
     * reconnect or a sync can time out, and it then shows the job as stopped.
     * Restarting here would reset the trip's figures and write a second
     * TRIP_START, and the phone's parser drops every row before the last one
     * from the job. Keep the trip running and confirm it instead.
     */
    if (s_trip_active) {
        ESP_LOGW(TAG, "TRIP_START with a trip already running - kept it");
        marker_done(true);
        return;
    }

    // Normally the pair is already open. If it is not (no card, or the last
    // rotation could not open the next file), try once more now.
    if (!s_file && !open_log()) {
        marker_done(false);   // nothing to record the trip in; tell the phone
        return;
    }
    if (!s_snap) open_snap();

    vehicle_state_t st;
    vehicle_state_snapshot(&st);

    s_trip_start_ah     = st.latest.isa_ah;
    s_trip_start_kwh    = st.latest.isa_kwh;
    s_trip_start_soc    = st.latest.soc;
    s_trip_start_tick   = (uint32_t)(esp_timer_get_time() / 1000);
    s_trip_peak_current = 0;
    s_trip_peak_rpm     = 0;
    s_trip_peak_motor_c = 0;
    s_trip_peak_inv_c   = 0;
    s_trip_peak_bms_c   = 0;
    s_trip_active       = true;

    write_trip_start_rows();

    marker_done(true);
    ESP_LOGI(TAG, "Trip started - soc=%u%%", s_trip_start_soc);
}

/*
 * Close both files and rotate to a new session straight away, so the finished
 * session can be listed and fetched by the phone while the logger keeps
 * recording into the next one.
 *
 * The next pair is opened here, before the phone is answered. Leaving it to
 * be reopened later meant nothing was recorded until the 30 s status pass
 * noticed, and that pass then claimed yet another number, skipping this one.
 */
static void handle_trip_end(void)
{
    if (!s_trip_active) {
        ESP_LOGW(TAG, "TRIP_END with no active trip - ignored");
        marker_done(true);   // as before: nothing to end is not an error
        return;
    }
    s_trip_active = false;

    vehicle_state_t st;
    vehicle_state_snapshot(&st);

    uint32_t end_tick   = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t duration_s = (end_tick - s_trip_start_tick) / 1000;
    float    ah_used    = (st.latest.isa_ah  - s_trip_start_ah)  / 3600.0f;
    float    kwh_used   = (st.latest.isa_kwh - s_trip_start_kwh) / 1000.0f;
    uint8_t  soc_end    = st.latest.soc;
    float    peak_a     = s_trip_peak_current / 1000.0f;

    // The RTC gives a real start time when it is set; the telematics build had
    // to derive this from the phone's offset.
    uint32_t start_unix = 0;
    if (rtc_time_valid()) {
        time_t now = time(NULL);
        start_unix = (uint32_t)(now - (time_t)duration_s);
    }

    if (s_snap) {
        fprintf(s_snap,
            "TRIP_END,%lu,%.2f,%.3f,%u,%u,%.1f,%lu,%d,%d,%d,%d,,,,\n",
            (unsigned long)duration_s, ah_used, kwh_used,
            s_trip_start_soc, soc_end, peak_a,
            (unsigned long)start_unix,
            (int)s_trip_peak_rpm, (int)s_trip_peak_motor_c,
            (int)s_trip_peak_inv_c, (int)s_trip_peak_bms_c);
        fflush(s_snap);
        fsync(fileno(s_snap));
        fclose(s_snap);
        s_snap = NULL;
    }

    if (s_file) {
        fprintf(s_file, "TRIP_END,%lu,%.2f,%.3f,%u,%u,%.1f\n",
                (unsigned long)duration_s, ah_used, kwh_used,
                s_trip_start_soc, soc_end, peak_a);
        fflush(s_file);
        fsync(fileno(s_file));
        fclose(s_file);
        s_file = NULL;

        s_log_id = claim_log_id();
        sd_store_reclaim(s_want_free, s_log_id);
        if (open_log()) {
            open_snap();
        } else {
            // The status pass keeps retrying under a fresh number.
            ESP_LOGE(TAG, "Could not open the next session after TRIP_END");
        }
    }

    // Let the BLE task reply only once the rotation is really done. The trip
    // itself was closed out either way, so this reports success.
    marker_done(true);

    ESP_LOGI(TAG, "Trip ended - %lus, %.2fAh, %.3fkWh, SoC %u%%->%u%%, peak %.1fA",
             (unsigned long)duration_s, ah_used, kwh_used,
             s_trip_start_soc, soc_end, peak_a);
}

// ── Task ──────────────────────────────────────────────────────────────────────

void can_logger_task(void *pvParameters)
{
    (void)pvParameters;

    can_decode_init();

    // Created before the CAN peripheral starts, so no frame can arrive while
    // the queue handle is still NULL. Internal RAM explicitly: with
    // SPIRAM_USE_MALLOC a buffer this size would otherwise land in PSRAM.
    s_frame_queue = xQueueCreateWithCaps(FRAME_QUEUE_DEPTH, sizeof(can_frame_t),
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    configASSERT(s_frame_queue);

    // Created before BLE can post to them.
    s_marker_queue = xQueueCreate(4, sizeof(trip_marker_t));
    s_marker_sem   = xSemaphoreCreateBinary();
    configASSERT(s_marker_queue && s_marker_sem);

    // DMA-capable and cache-line aligned, so FATFS's whole-sector writes can
    // go straight from this buffer to the SDMMC DMA without a bounce copy.
    s_filebuf = heap_caps_aligned_alloc(64, FILE_BUF_BYTES,
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!s_filebuf) ESP_LOGW(TAG, "No memory for file buffer - using default");

    while (!sd_store_mount()) {
        ESP_LOGW(TAG, "SD mount failed - retrying in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    s_want_free = sd_store_total_bytes() * FREE_SPACE_PCT / 100;
    s_log_id    = claim_log_id();

    // Make room before opening, so a nearly full card cannot fail the open.
    sd_store_reclaim(s_want_free, s_log_id);

    /*
     * Bounded, unlike the mount loop above. An open can fail transiently (card
     * busy) or permanently (write-protected card, corrupt FAT, a full FAT16
     * root directory — plausible here, since the log number keeps climbing
     * while retention deletes by lowest number). Retrying a permanent failure
     * forever would mean never reaching can_handler_start() below, so the
     * device would sit silent with no CAN, no log and no status output, which
     * is indistinguishable from dead hardware.
     *
     * Try a remount once partway through, then give up on the file and carry on
     * regardless: with CAN up and the status loop running, the fault is at
     * least diagnosable from the console.
     */
    bool logging = false;
    for (int attempt = 0; attempt < OPEN_RETRY_LIMIT; attempt++) {
        if (open_log()) { open_snap(); logging = true; break; }

        if (attempt == OPEN_RETRY_LIMIT / 2) {
            ESP_LOGW(TAG, "Open still failing - remounting card");
            remount_card();
        }
        vTaskDelay(pdMS_TO_TICKS(OPEN_RETRY_MS));
    }

    if (!logging) {
        ESP_LOGE(TAG, "Could not open a log file after %d attempts - "
                      "continuing without logging so the fault is visible",
                 OPEN_RETRY_LIMIT);
    }

    // Frames are only accepted once the file is open and its header committed.
    if (!can_handler_start()) {
        ESP_LOGE(TAG, "CAN start failed - logger idle");
    }

    int64_t last_flush   = esp_timer_get_time();
    int64_t last_space   = last_flush;
    int64_t last_status  = last_flush;
    int64_t last_snap    = last_flush;
    int64_t last_recover = 0;   // 0 => first bus-off recovers immediately

    can_frame_t f;
    while (1) {
        /*
         * Drain in a tight inner loop. The housekeeping below (a timer read
         * and several 64-bit comparisons) costs more than formatting a frame,
         * so running it per-frame stops the task keeping up with a busy bus
         * and the ISR starts dropping. Batch the drain instead, and bound the
         * batch so the flush cadence still holds under sustained load.
         */
        refresh_unix_offset();

        int batch = 0;
        while (batch < DRAIN_BATCH_MAX &&
               xQueueReceive(s_frame_queue, &f,
                             batch ? 0 : pdMS_TO_TICKS(100)) == pdTRUE) {
            /*
             * Decode before writing. This task is the single writer of
             * vehicle_state, and decoding here means the dash sees a value the
             * instant the frame is logged. The lock is taken per frame, never
             * per batch, so interrupts on this core are masked only for
             * microseconds at a time.
             */
            raw_can_log_t rf = {
                .tick_ms = (uint32_t)(f.us / 1000),
                .id      = f.id,
                .dlc     = f.dlc,
            };
            memcpy(rf.data, f.data, sizeof(rf.data));
            vehicle_state_lock();
            can_decode_frame(&rf);
            vehicle_state_unlock();

            // Speedo mode streams raw frames to the phone. Non-blocking: the
            // BLE link cannot carry a busy bus, and the log must never wait
            // on it.
            if (g_app_mode == APP_MODE_SPEEDO && g_ble_live_queue) {
                xQueueSend(g_ble_live_queue, &rf, 0);
            }

            write_frame(&f);
            batch++;
        }

        int64_t now = esp_timer_get_time();

        // ── Trip markers from the phone ─────────────────────────────────────
        trip_marker_t marker;
        while (xQueueReceive(s_marker_queue, &marker, 0) == pdTRUE) {
            if (marker == TRIP_MARKER_START) handle_trip_start();
            else                             handle_trip_end();
        }

        // ── 1 Hz snapshot ───────────────────────────────────────────────────
        if (s_snap && now - last_snap >= (int64_t)SNAP_INTERVAL_MS * 1000) {
            last_snap = now;
            write_snapshot((uint32_t)(now / 1000));
        }

        if (now - last_flush >= (int64_t)FLUSH_INTERVAL_MS * 1000) {
            note_drops();

            if (s_file) {
                if (commit()) {
                    s_commit_fails = 0;
                } else if (++s_commit_fails >= COMMIT_FAIL_LIMIT) {
                    /*
                     * The stream is not accepting writes any more. Everything
                     * written since the last good commit is already lost, and
                     * staying on this FILE* would discard everything after it
                     * too. Start a new file; if that also fails, fall back to
                     * remounting on the next pass.
                     */
                    ESP_LOGE(TAG, "canlog_%04lu.csv failed %lu commits - "
                                  "abandoning it and opening a new file",
                             (unsigned long)s_log_id,
                             (unsigned long)s_commit_fails);
                    if (!reopen_log()) {
                        ESP_LOGE(TAG, "Reopen failed - remounting card");
                        remount_card();   // both files are closed by now
                    }
                    s_commit_fails = 0;
                }
            }

            last_flush = now;
        }

        /*
         * Recovery is only *started* by can_handler_recover(); the hardware
         * needs 128 consecutive recessive bit-times before it is active again,
         * and that completion arrives later as a state-change callback which
         * clears the live bus-off state. So the node coming back on the bus —
         * not the return of the recover call — is what retires the request.
         *
         * Retrying while still bus-off is harmless and necessary: the driver
         * rejects recover() unless the node is currently bus-off, so a request
         * that lost the race would otherwise be dropped and the logger would
         * stay deaf for the rest of the power cycle.
         */
        if (s_bus_off) {
            if (!can_handler_is_bus_off()) {
                s_bus_off = false;          // recovery completed
                ESP_LOGW(TAG, "Bus-off recovered - receiving again");
            } else if (now - last_recover >= (int64_t)RECOVER_RETRY_MS * 1000) {
                last_recover = now;
                can_handler_recover();
            }
        }

        if (now - last_space >= (int64_t)SPACE_CHECK_INTERVAL_MS * 1000) {
            last_space = now;
            if (sd_store_free_bytes() < s_want_free) {
                sd_store_reclaim(s_want_free, s_log_id);
            }
        }

        if (now - last_status >= (int64_t)STATUS_INTERVAL_MS * 1000) {
            last_status = now;

            /*
             * Report the two states that mean "recording nothing" explicitly.
             * Counting bus-off events is not enough: a node that went off once
             * and never recovered looks identical to one that recovered fine.
             */
            ESP_LOGI(TAG,
                     "canlog_%04lu.csv: %lu frames, %lu dropped, "
                     "%lu bus errors, %lu bus-off%s, %lu reopens, %lluMB free%s",
                     (unsigned long)s_log_id,
                     (unsigned long)s_written,
                     (unsigned long)s_drops,
                     (unsigned long)can_handler_error_count(),
                     (unsigned long)can_handler_bus_off_count(),
                     can_handler_is_bus_off() ? " (CURRENTLY BUS-OFF)" : "",
                     (unsigned long)s_reopens,
                     sd_store_free_bytes() >> 20,
                     s_file ? "" : " - NOT LOGGING");

            /*
             * No file: an earlier open failed. Keep trying at the status
             * cadence rather than spinning. Claim a fresh number each time via
             * reopen_log() — the card may have been swapped, and the old number
             * could now collide with a file already on the new card.
             */
            if (!s_file) {
                if (reopen_log()) {
                    ESP_LOGW(TAG, "Logging resumed as canlog_%04lu.csv",
                             (unsigned long)s_log_id);
                } else {
                    // Maybe the card was pulled and put back. Pick it up
                    // now, so the next pass can open a file on it.
                    remount_card();
                }
            }
        }
    }
}
