#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/*
 * can_logger.h — SD card CAN log writer
 *
 * Logging starts automatically at boot and never stops: there is no operator
 * and no control channel. Power is cut without warning, so the log is flushed
 * and fsync'd on a fixed cadence rather than at close.
 */

// One captured frame, as handed over from the CAN ISR.
typedef struct {
    int64_t  us;        // microseconds since boot, stamped at receive time
    uint32_t id;        // 11-bit or 29-bit arbitration ID
    uint8_t  dlc;       // payload length, 0..8
    uint8_t  ext;       // 1 = 29-bit extended ID
    uint8_t  rtr;       // 1 = remote frame (no payload)
    uint8_t  data[8];
} can_frame_t;

// Core-0 task: owns the SD card and the log file, and does all file I/O.
void can_logger_task(void *pvParameters);

// Queue a frame from the CAN ISR. Non-blocking; drops (and counts) on overflow.
void can_logger_submit_from_isr(const can_frame_t *f, BaseType_t *woken);

// Flag a bus-off for the task to recover from. ISR-safe.
void can_logger_notify_bus_off_from_isr(void);

// ── Trip markers, posted by the BLE task ─────────────────────────────────────
//
// The phone starts and ends trips. TRIP_END closes both files and rotates to a
// new session immediately, so the finished session can be listed and fetched.

typedef enum {
    TRIP_MARKER_START,
    TRIP_MARKER_END,
} trip_marker_t;

// Post a marker to the logger task. Returns false if the queue is full.
// Clears any stale completion first, so a following can_logger_wait_marker()
// waits for *this* marker rather than one whose earlier wait timed out.
bool can_logger_post_marker(trip_marker_t type);

/*
 * Block until the logger has handled the posted marker, so the caller replies
 * to the phone only once it has really taken effect (for TRIP_END, the new
 * session is open). *ok is set to whether the marker reached a log file.
 * Returns false on timeout, leaving *ok untouched.
 */
bool can_logger_wait_marker(uint32_t timeout_ms, bool *ok);

// Queue a frame from task context. Bench simulation only: it takes the same
// path as a real frame, so simulated data is decoded *and* logged.
void can_logger_submit(const can_frame_t *f);

// Current session number, for the BLE STATUS reply.
uint32_t can_logger_session_id(void);

// True while a trip is running.
bool can_logger_trip_active(void);

// True while a log file is open and accepting writes.
bool can_logger_is_logging(void);
