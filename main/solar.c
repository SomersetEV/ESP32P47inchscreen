#include <math.h>

#include "solar.h"

/*
 * NOAA low-precision sunrise/sunset algorithm (the same formulas behind
 * NOAA's public sunrise/sunset calculator spreadsheet). Accurate to within
 * about a minute for non-polar latitudes, which is all a backlight fade
 * needs.
 */

#define DEG2RAD(d) ((d) * M_PI / 180.0)
#define RAD2DEG(r) ((r) * 180.0 / M_PI)

// Solar zenith angle for the "official" sunrise/sunset definition (center of
// the sun 0.833 deg below a level horizon, accounting for atmospheric
// refraction and the sun's apparent radius).
#define ZENITH_DEG 90.833

bool solar_sunrise_sunset(time_t now_utc, double lat_deg, double lon_deg,
                          time_t *sunrise_utc, time_t *sunset_utc)
{
    struct tm day;
    gmtime_r(&now_utc, &day);

    // Julian day number for this UTC calendar day at 12:00.
    int year  = day.tm_year + 1900;
    int month = day.tm_mon + 1;
    int dom   = day.tm_mday;

    int a = (14 - month) / 12;
    int y = year + 4800 - a;
    int m = month + 12 * a - 3;
    long jdn = dom + (153 * m + 2) / 5 + 365L * y + y / 4 - y / 100 + y / 400 - 32045;

    // Julian century referenced to 2000-01-01 12:00 UTC (JD 2451545.0).
    double jc = ((double)jdn - 2451545.0) / 36525.0;

    double geom_mean_long_sun = fmod(280.46646 + jc * (36000.76983 + jc * 0.0003032), 360.0);
    double geom_mean_anom_sun = 357.52911 + jc * (35999.05029 - 0.0001537 * jc);
    double eccent_earth_orbit = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc);

    double sun_eq_of_ctr = sin(DEG2RAD(geom_mean_anom_sun)) * (1.914602 - jc * (0.004817 + 0.000014 * jc))
                          + sin(DEG2RAD(2 * geom_mean_anom_sun)) * (0.019993 - 0.000101 * jc)
                          + sin(DEG2RAD(3 * geom_mean_anom_sun)) * 0.000289;

    double sun_true_long = geom_mean_long_sun + sun_eq_of_ctr;
    double sun_app_long   = sun_true_long - 0.00569 - 0.00478 * sin(DEG2RAD(125.04 - 1934.136 * jc));

    double mean_obliq_ecliptic = 23.0 + (26.0 + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60.0) / 60.0;
    double obliq_corr = mean_obliq_ecliptic + 0.00256 * cos(DEG2RAD(125.04 - 1934.136 * jc));

    double sun_declin = RAD2DEG(asin(sin(DEG2RAD(obliq_corr)) * sin(DEG2RAD(sun_app_long))));

    double var_y = tan(DEG2RAD(obliq_corr / 2.0)) * tan(DEG2RAD(obliq_corr / 2.0));
    double eq_of_time = 4.0 * RAD2DEG(
          var_y * sin(2.0 * DEG2RAD(geom_mean_long_sun))
        - 2.0 * eccent_earth_orbit * sin(DEG2RAD(geom_mean_anom_sun))
        + 4.0 * eccent_earth_orbit * var_y * sin(DEG2RAD(geom_mean_anom_sun)) * cos(2.0 * DEG2RAD(geom_mean_long_sun))
        - 0.5 * var_y * var_y * sin(4.0 * DEG2RAD(geom_mean_long_sun))
        - 1.25 * eccent_earth_orbit * eccent_earth_orbit * sin(2.0 * DEG2RAD(geom_mean_anom_sun)));

    double cos_hour_angle = cos(DEG2RAD(ZENITH_DEG)) / (cos(DEG2RAD(lat_deg)) * cos(DEG2RAD(sun_declin)))
                           - tan(DEG2RAD(lat_deg)) * tan(DEG2RAD(sun_declin));
    if (cos_hour_angle < -1.0 || cos_hour_angle > 1.0) {
        // Sun never sets (< -1) or never rises (> 1) at this latitude/date.
        return false;
    }
    double hour_angle_deg = RAD2DEG(acos(cos_hour_angle));

    // Solar noon and sunrise/sunset, in minutes from UTC midnight.
    double solar_noon_min  = 720.0 - 4.0 * lon_deg - eq_of_time;
    double sunrise_min     = solar_noon_min - 4.0 * hour_angle_deg;
    double sunset_min      = solar_noon_min + 4.0 * hour_angle_deg;

    struct tm midnight = day;
    midnight.tm_hour = 0;
    midnight.tm_min  = 0;
    midnight.tm_sec  = 0;
    time_t midnight_utc = timegm(&midnight);

    *sunrise_utc = midnight_utc + (time_t)lround(sunrise_min * 60.0);
    *sunset_utc  = midnight_utc + (time_t)lround(sunset_min * 60.0);
    return true;
}
