#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * vehicle_state.h
 *
 * Latest decoded values from every CAN source. Written exclusively by the
 * logger task (which decodes each frame as it dequeues it); read by the UI,
 * the BLE task and the snapshot writer.
 *
 * log_record_t keeps the telematics field set and scaling unchanged, because
 * the phone app's SNAP1 parser depends on it. Dash-only values live in
 * vehicle_state_t alongside it so the snapshot layout is untouched.
 */

// ── Log record: written to SD and transmitted over BLE ───────────────────────
// Scaling is noted inline and must not change; the app relies on it.

typedef struct __attribute__((packed)) {
    uint32_t tick_ms;           // ms since boot
    int32_t  unix_offset;       // 0 until time is known, then (unix - tick_ms/1000)

    // ── ZombieVerter / Leaf inverter ─────────────────────────────────────────
    int16_t  motor_rpm;         // RPM, signed (negative = reverse)  — 0x1DA
    int16_t  motor_temp;        // °C x10                           — 0x55A
    int16_t  inverter_temp;     // °C x10                           — 0x55A

    // ── ISA IVT-S shunt (int32 big-endian in bytes 2..5) ────────────────────
    int32_t  pack_current_ma;   // mA  (/1000 = A)   — 0x521
    int32_t  pack_voltage_mv;   // mV  (/1000 = V)   — 0x522
    int32_t  isa_kw;            // W   (/1000 = kW)  — 0x526
    int32_t  isa_ah;            // As  (/3600 = Ah)  — 0x527
    int32_t  isa_kwh;           // Wh  (/1000 = kWh) — 0x528

    // ── M3 BMS ──────────────────────────────────────────────────────────────
    uint8_t  soc;               // %                 — 0x355 byte 0
    int16_t  bms_temp_max;      // °C x10            — 0x356
    int16_t  bms_temp_min;      // °C x10
    uint16_t cell_voltage_max;  // mV                — 0x373 bytes 2-3
    uint16_t cell_voltage_min;  // mV                — 0x373 bytes 0-1
    uint16_t pack_voltage_bms;  // 0.01 V units      — 0x356 bytes 0-1

    // ── MG Gen2 V2L charger ─────────────────────────────────────────────────
    uint16_t lv_volts_mv;       // mV                — 0x39F
    uint8_t  lv_amps;           // A
    uint8_t  plug_state;        // 0 = unplugged, 1 = plugged
    int16_t  charger_temp;      // °C x10            — 0x33B

} log_record_t;

// ── Raw CAN frame for high-rate logging ──────────────────────────────────────
typedef struct {
    uint32_t tick_ms;
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} raw_can_log_t;

// ── Live state ───────────────────────────────────────────────────────────────
// Dash-only derived values sit here rather than in log_record_t so the SNAP1
// column layout the phone app parses stays exactly as it was.

typedef struct {
    log_record_t latest;

    int32_t  power_w;       // pack_v * amps, computed on decode (negative = regen)
    int16_t  speed_mph_x10; // derived from motor_rpm alone, no wheel sensor
    uint16_t cell_delta_mv; // cell_voltage_max - cell_voltage_min

    uint64_t last_frame_us; // esp_timer time of the most recent frame, 0 if none
    uint8_t  can_rx_errors;
} vehicle_state_t;

void vehicle_state_init(void);
vehicle_state_t *vehicle_state_get(void);

/*
 * Copy the whole struct out under a short critical section. Readers that need
 * a coherent set of values (the UI, BLE summaries) must use this rather than
 * dereferencing vehicle_state_get() field by field.
 */
void vehicle_state_snapshot(vehicle_state_t *out);

// True when a frame has arrived within the last stale_ms. The dash blanks every
// value to "--" when this goes false.
bool vehicle_state_is_fresh(uint32_t stale_ms);
