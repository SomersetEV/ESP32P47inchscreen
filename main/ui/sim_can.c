#include "sdkconfig.h"

#if CONFIG_DASH_SIMULATE_CAN

#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "can_decode.h"
#include "sim_can.h"

/*
 * Bench-only synthetic CAN.
 *
 * Builds frames for every ID the dash decodes and pushes them through
 * can_decode_frame(), the same entry point the real bus uses, so what is
 * exercised here is the real decode and display path — not a parallel one.
 *
 * Values sweep on a slow sine so every widget visibly changes, including the
 * colour thresholds and the regen (negative kW) case.
 */

static const char *TAG = "SIM";
static esp_timer_handle_t s_timer;

#define SIM_PERIOD_US  50000   // 50 ms

static void put_isa(uint8_t *d, int32_t v)
{
    // ISA packs its int32 into bytes 2..5, LSB first.
    d[2] = (uint8_t)(v & 0xff);
    d[3] = (uint8_t)((v >> 8) & 0xff);
    d[4] = (uint8_t)((v >> 16) & 0xff);
    d[5] = (uint8_t)((v >> 24) & 0xff);
}

static void emit(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    raw_can_log_t f = { .tick_ms = (uint32_t)(esp_timer_get_time() / 1000),
                        .id = id, .dlc = dlc };
    memcpy(f.data, data, dlc);
    can_decode_frame(&f);
}

static void sim_cb(void *arg)
{
    (void)arg;
    static float phase = 0.0f;
    phase += 0.02f;

    const float s = sinf(phase);              // -1 .. 1
    const float u = (s + 1.0f) * 0.5f;        //  0 .. 1

    uint8_t d[8] = {0};

    // 0x1DA — motor RPM, swings through zero so speed and regen both show.
    int16_t rpm = (int16_t)(s * 3000.0f);
    int16_t raw_rpm = rpm * 2;
    d[4] = (uint8_t)((raw_rpm >> 8) & 0xff);
    d[5] = (uint8_t)(raw_rpm & 0xff);
    emit(0x1DA, d, 8);

    // 0x55A — motor / inverter temperature in °F, sweeping past the amber and
    // red thresholds (60 / 80 °C).
    memset(d, 0, sizeof(d));
    d[1] = (uint8_t)(100 + u * 100);          // motor
    d[2] = (uint8_t)(90  + u * 100);          // inverter
    emit(0x55A, d, 8);

    // 0x355 — SOC, dips below the 15 % alert threshold.
    memset(d, 0, sizeof(d));
    d[0] = (uint8_t)(u * 100.0f);
    emit(0x355, d, 8);

    // 0x356 — pack temperature, 0.1 °C units in bytes 4-5.
    memset(d, 0, sizeof(d));
    int16_t bt = (int16_t)(200 + u * 700);
    d[4] = (uint8_t)(bt & 0xff);
    d[5] = (uint8_t)((bt >> 8) & 0xff);
    emit(0x356, d, 8);

    // 0x373 — cell min / max in mV.
    memset(d, 0, sizeof(d));
    uint16_t cmin = (uint16_t)(3600 + u * 100);
    uint16_t cmax = cmin + (uint16_t)(20 + u * 60);
    d[0] = cmin & 0xff; d[1] = cmin >> 8;
    d[2] = cmax & 0xff; d[3] = cmax >> 8;
    emit(0x373, d, 8);

    // 0x522 — pack voltage, mV. Decoded before current so kW is consistent.
    memset(d, 0, sizeof(d));
    put_isa(d, (int32_t)(350000 + u * 20000));
    emit(0x522, d, 8);

    // 0x521 — pack current. Sign follows RPM so braking shows regen green.
    memset(d, 0, sizeof(d));
    put_isa(d, (int32_t)(-s * 15000.0f));
    emit(0x521, d, 8);

    // 0x33B — charger temperature (raw - 40).
    memset(d, 0, sizeof(d));
    d[3] = (uint8_t)(60 + u * 60);
    emit(0x33B, d, 8);

    // 0x39F — aux 12 V rail (raw / 8).
    memset(d, 0, sizeof(d));
    d[1] = (uint8_t)(104 + u * 8);            // ~13 - 14 V
    emit(0x39F, d, 8);
}

void sim_can_start(void)
{
    const esp_timer_create_args_t args = {
        .callback = sim_cb,
        .name     = "sim_can",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, SIM_PERIOD_US));
    ESP_LOGW(TAG, "SIMULATED CAN ACTIVE - bench only, values are not real");
}

#else  /* !CONFIG_DASH_SIMULATE_CAN */

void sim_can_start(void) { }

#endif
