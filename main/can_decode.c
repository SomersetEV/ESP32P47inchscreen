#include <stdlib.h>
#include "esp_timer.h"
#include "sdkconfig.h"

#include "can_decode.h"

/*
 * Formulas are lifted from Leyland_255_Nextion_dash.ino so the new dash shows
 * the same numbers as the old one. Where that sketch used floats and °C, the
 * values are stored as °C x10 to match the telematics log_record_t scaling the
 * phone app parses.
 *
 * Speed is the one deliberate change: the Nextion dash took mph from GPS, and
 * there is no GPS here, so it is derived from motor RPM alone.
 */

#ifndef CONFIG_DASH_CURRENT_DIVISOR
#define CONFIG_DASH_CURRENT_DIVISOR 100
#endif
#ifndef CONFIG_DASH_MPH_PER_RPM
#define CONFIG_DASH_MPH_PER_RPM "0.008"
#endif

// The ISA shunt packs a signed 32-bit value into bytes 2..5, least significant
// byte first (byte[5] is the MSB in the sketch's shift order).
static inline int32_t isa_int32(const uint8_t *d)
{
    return (int32_t)(((uint32_t)d[5] << 24) | ((uint32_t)d[4] << 16) |
                     ((uint32_t)d[3] << 8)  |  (uint32_t)d[2]);
}

/*
 * DASH_MPH_PER_RPM is a Kconfig string, parsed once here rather than per 0x1DA
 * frame. It cannot be parsed lazily inside can_decode_frame(): that runs under
 * the vehicle_state critical section, and strtod may allocate.
 */
static float s_mph_per_rpm;

void can_decode_init(void)
{
    s_mph_per_rpm = (float)atof(CONFIG_DASH_MPH_PER_RPM);
}

void can_decode_frame(const raw_can_log_t *f)
{
    vehicle_state_t *s = vehicle_state_get();
    log_record_t    *r = &s->latest;
    const uint8_t   *d = f->data;

    // Any frame at all counts as the bus being alive, even an ID we ignore.
    s->last_frame_us = esp_timer_get_time();
    r->tick_ms = f->tick_ms;

    switch (f->id) {

    case 0x1DA:   // Leaf inverter: motor RPM
        if (f->dlc >= 6) {
            int16_t raw = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
            r->motor_rpm = raw / 2;

            // mph = rpm * DASH_MPH_PER_RPM, kept as x10 for one decimal place.
            float mph = (float)r->motor_rpm * s_mph_per_rpm;
            s->speed_mph_x10 = (int16_t)(mph * 10.0f);
        }
        break;

    case 0x55A:   // Leaf inverter: motor and inverter temperature, °F raw
        if (f->dlc >= 3) {
            r->motor_temp    = (int16_t)((((float)d[1] - 32.0f) / 1.8f) * 10.0f);
            r->inverter_temp = (int16_t)((((float)d[2] - 32.0f) / 1.8f) * 10.0f);
        }
        break;

    case 0x355:   // M3 BMS: state of charge
        if (f->dlc >= 1) {
            r->soc = d[0];
        }
        break;

    case 0x356:   // M3 BMS: pack voltage (0.01 V) and temperature (0.1 °C)
        if (f->dlc >= 6) {
            // Bytes 0-1, little-endian, as the telematics firmware decoded it.
            // Feeds the SNAP1 pack_v_bms_mv column, which was otherwise always 0.
            r->pack_voltage_bms = (uint16_t)(((uint16_t)d[1] << 8) | d[0]);

            int16_t raw = (int16_t)(((uint16_t)d[5] << 8) | d[4]);
            // The sketch showed a single pack temperature; the log record keeps
            // the telematics min/max pair, so both carry the same value.
            r->bms_temp_max = raw;
            r->bms_temp_min = raw;
        }
        break;

    case 0x373:   // M3 BMS (SIMP format): cell min/max in mV
        if (f->dlc >= 4) {
            r->cell_voltage_min = (uint16_t)(((uint16_t)d[1] << 8) | d[0]);
            r->cell_voltage_max = (uint16_t)(((uint16_t)d[3] << 8) | d[2]);
            s->cell_delta_mv    = (r->cell_voltage_max > r->cell_voltage_min)
                                ? (uint16_t)(r->cell_voltage_max - r->cell_voltage_min)
                                : 0;
        }
        break;

    case 0x522:   // ISA shunt: pack voltage, mV
        if (f->dlc >= 6) {
            r->pack_voltage_mv = isa_int32(d);
        }
        break;

    case 0x521:   // ISA shunt: pack current
        if (f->dlc >= 6) {
            // The sketch negates the raw value so discharge reads positive,
            // then divides by 100 to get amps. Stored here in mA.
            int32_t raw = -isa_int32(d);
            r->pack_current_ma = (raw * 1000) / CONFIG_DASH_CURRENT_DIVISOR;

            // kW = pack_v * amps / 1000, computed in W to stay integer.
            int64_t w = ((int64_t)r->pack_voltage_mv * (int64_t)r->pack_current_ma)
                      / 1000000LL;
            s->power_w = (int32_t)w;
        }
        break;

    case 0x526:   // ISA shunt: power, W
        if (f->dlc >= 6) { r->isa_kw = isa_int32(d); }
        break;

    case 0x527:   // ISA shunt: charge counter, As
        if (f->dlc >= 6) { r->isa_ah = isa_int32(d); }
        break;

    case 0x528:   // ISA shunt: energy counter, Wh
        if (f->dlc >= 6) { r->isa_kwh = isa_int32(d); }
        break;

    case 0x33B:   // MG charger temperature
        // The Nextion sketch used -40 here; the telematics firmware used -50 on
        // the same ID. The dash follows the sketch, which is what the tractor
        // was actually reading.
        if (f->dlc >= 4) {
            r->charger_temp = (int16_t)(((int)d[3] - 40) * 10);
        }
        break;

    case 0x39F:   // MG charger: aux 12 V rail
        if (f->dlc >= 2) {
            // Sketch reads bytes[1] / 8 as volts; stored as mV.
            r->lv_volts_mv = (uint16_t)(((uint32_t)d[1] * 1000u) / 8u);
        }
        break;

    default:
        break;    // not displayed, but still logged raw
    }
}
