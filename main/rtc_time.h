#pragma once
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

/*
 * rtc_time.h — battery-backed wall clock.
 *
 * The board carries a PCF85063A on the shared I2C bus with a 1220 coin cell,
 * so logs can carry real dates across power cycles. Nothing else depends on
 * it: if the part is missing or never set, the firmware runs with time
 * invalid, the dash shows "--:--" and the unix columns in the logs stay 0.
 */

// Probe the RTC and, if it holds a plausible time, seed the system clock from
// it. Safe to call when the part is absent. Must run after bsp_i2c_init().
esp_err_t rtc_time_init(void);

// Set both the system clock and the RTC. Called by the phone's TIME command.
esp_err_t rtc_time_set(time_t unix_epoch);

// True once the clock has been seeded from the RTC or set by the phone.
bool rtc_time_valid(void);
