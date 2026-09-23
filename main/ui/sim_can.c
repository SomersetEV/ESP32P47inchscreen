#include "sdkconfig.h"

#if CONFIG_DASH_SIMULATE_CAN

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
 * Currently pinned to a fixed scenario (constant RPM/speed/power, everything
 * else static) rather than the usual sine sweep — see the constants below.
 * Speed is NOT set directly: it is derived from RPM by the same
 * DASH_MPH_PER_RPM ratio the real bus decode uses, so it follows RPM rather
 * than being an independent knob.
 */

// 2200 rpm constant. 0x1DA carries raw = rpm * 2.
#define SIM_MOTOR_RPM     2200
// 15 kW constant, built from a fixed pack voltage/current pair whose product
// is 15000 W (power_w = pack_voltage_mv * pack_current_ma / 1e6).
#define SIM_PACK_VOLTAGE_MV     360000  // 360 V
// can_decode does pack_current_ma = raw_isa_current * 1000 / DASH_CURRENT_DIVISOR
// (default divisor 100, so raw is in centiamps: pack_current_ma = raw * 10).
// raw = 4167 -> 41.67 A -> 360 V * 41.67 A = 15.0 kW.
#define SIM_PACK_CURRENT_RAW      4167

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

    uint8_t d[8] = {0};

    // 0x1DA — motor RPM, held at SIM_MOTOR_RPM. Speed on screen follows from
    // this via DASH_MPH_PER_RPM, it is not set separately.
    int16_t raw_rpm = (int16_t)(SIM_MOTOR_RPM * 2);
    d[4] = (uint8_t)((raw_rpm >> 8) & 0xff);
    d[5] = (uint8_t)(raw_rpm & 0xff);
    emit(0x1DA, d, 8);

    // 0x55A — motor / inverter temperature in °F. Held mid-range, well clear
    // of the amber/red thresholds (60/80 °C).
    memset(d, 0, sizeof(d));
    d[1] = 100;                               // motor
    d[2] = 90;                                // inverter
    emit(0x55A, d, 8);

    // 0x355 — SOC, held at 50 %.
    memset(d, 0, sizeof(d));
    d[0] = 50;
    emit(0x355, d, 8);

    // 0x356 — pack temperature, 0.1 °C units in bytes 4-5. Held at 25.0 °C.
    memset(d, 0, sizeof(d));
    int16_t bt = 250;
    d[4] = (uint8_t)(bt & 0xff);
    d[5] = (uint8_t)((bt >> 8) & 0xff);
    emit(0x356, d, 8);

    // 0x373 — cell min / max in mV. Held at a plausible mid-charge pair.
    memset(d, 0, sizeof(d));
    uint16_t cmin = 3650;
    uint16_t cmax = 3680;
    d[0] = cmin & 0xff; d[1] = cmin >> 8;
    d[2] = cmax & 0xff; d[3] = cmax >> 8;
    emit(0x373, d, 8);

    // 0x522 — pack voltage, mV. Decoded before current so kW is consistent.
    memset(d, 0, sizeof(d));
    put_isa(d, SIM_PACK_VOLTAGE_MV);
    emit(0x522, d, 8);

    // 0x521 — pack current. can_decode negates this on the way in, so send
    // the negated value here to land on +SIM_PACK_CURRENT_RAW (discharge).
    memset(d, 0, sizeof(d));
    put_isa(d, -SIM_PACK_CURRENT_RAW);
    emit(0x521, d, 8);

    // 0x33B — charger temperature (raw - 40). Held at 20.0 °C.
    memset(d, 0, sizeof(d));
    d[3] = 60;
    emit(0x33B, d, 8);

    // 0x39F — aux 12 V rail (raw / 8). Held at ~13.5 V.
    memset(d, 0, sizeof(d));
    d[1] = 108;
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
