#pragma once
#include <time.h>
#include <stdbool.h>

/*
 * solar.h — low-precision sunrise/sunset (NOAA/Meeus algorithm).
 *
 * Pure math, no I/O, no allocation, no ESP-IDF dependency beyond <time.h> —
 * deliberately kept freestanding so it can be compiled and sanity-checked
 * with a plain host compiler, separately from the firmware build.
 */

// Computes sunrise/sunset (UTC) for the given latitude/longitude on the UTC
// calendar day containing now_utc. Returns false if the sun does not
// rise/set that day (polar conditions) — not expected at UK latitudes, but
// checked rather than assumed, since a fat-fingered lat/long could land
// somewhere that genuinely has them.
bool solar_sunrise_sunset(time_t now_utc, double lat_deg, double lon_deg,
                          time_t *sunrise_utc, time_t *sunset_utc);
