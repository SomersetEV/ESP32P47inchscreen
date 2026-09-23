#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "bsp/esp-bsp.h"

#include "solar.h"
#include "rtc_time.h"
#include "ui_brightness.h"

/*
 * Sunrise/sunset-driven backlight. See ui_brightness.h.
 *
 * Deliberately a separate, permanent lv_timer rather than a reuse of
 * ui.c's boot-only backlight_ramp_cb: that timer is a one-shot 0->100%
 * animation that self-deletes once done, while this one runs forever,
 * computing a continuously moving target twice a day. They share the same
 * small-step style, not the same code.
 */

static const char *TAG = "BRIGHT";

#ifndef CONFIG_DASH_LATITUDE
#define CONFIG_DASH_LATITUDE "51.05"
#endif
#ifndef CONFIG_DASH_LONGITUDE
#define CONFIG_DASH_LONGITUDE "-2.85"
#endif
#ifndef CONFIG_DASH_BRIGHTNESS_DAY_PCT
#define CONFIG_DASH_BRIGHTNESS_DAY_PCT 100
#endif
#ifndef CONFIG_DASH_BRIGHTNESS_NIGHT_PCT
#define CONFIG_DASH_BRIGHTNESS_NIGHT_PCT 40
#endif
#ifndef CONFIG_DASH_BRIGHTNESS_FADE_MINUTES
#define CONFIG_DASH_BRIGHTNESS_FADE_MINUTES 30
#endif

#define TICK_MS          60000  // recheck every 60 s
#define MAX_STEP_PCT      3     // largest change applied per tick

static int    s_current_pct = 100;   // mirrors what's actually driven to LEDC
static time_t s_cached_day  = -1;    // UTC midnight the cached times are for
static time_t s_sunrise_utc, s_sunset_utc;
static bool   s_solar_valid;

// Linear fade between night% and day% across a FADE_MINUTES window centred
// on `event_utc` (sunrise or sunset). `rising` is true for sunrise (fading
// night->day), false for sunset (fading day->night).
static int fade_towards(time_t now, time_t event_utc, bool rising)
{
    const double half_window_s = CONFIG_DASH_BRIGHTNESS_FADE_MINUTES * 60.0;
    double frac = ((double)now - (double)event_utc + half_window_s) / (2.0 * half_window_s);
    if (frac < 0.0) frac = 0.0;
    if (frac > 1.0) frac = 1.0;

    int from = rising ? CONFIG_DASH_BRIGHTNESS_NIGHT_PCT : CONFIG_DASH_BRIGHTNESS_DAY_PCT;
    int to   = rising ? CONFIG_DASH_BRIGHTNESS_DAY_PCT   : CONFIG_DASH_BRIGHTNESS_NIGHT_PCT;
    return (int)lround(from + frac * (to - from));
}

// Ideal brightness for `now`, given the day's cached sunrise/sunset.
static int target_brightness(time_t now)
{
    const time_t half_window_s = (time_t)(CONFIG_DASH_BRIGHTNESS_FADE_MINUTES * 60);

    if (now < s_sunrise_utc - half_window_s) return CONFIG_DASH_BRIGHTNESS_NIGHT_PCT;
    if (now < s_sunrise_utc + half_window_s) return fade_towards(now, s_sunrise_utc, true);
    if (now < s_sunset_utc  - half_window_s) return CONFIG_DASH_BRIGHTNESS_DAY_PCT;
    if (now < s_sunset_utc  + half_window_s) return fade_towards(now, s_sunset_utc, false);
    return CONFIG_DASH_BRIGHTNESS_NIGHT_PCT;
}

static void step_towards(int target)
{
    if (s_current_pct == target) return;

    int delta = target - s_current_pct;
    if (delta > MAX_STEP_PCT)  delta = MAX_STEP_PCT;
    if (delta < -MAX_STEP_PCT) delta = -MAX_STEP_PCT;

    s_current_pct += delta;
    bsp_display_brightness_set(s_current_pct);
}

static void brightness_tick_cb(lv_timer_t *t)
{
    (void)t;

    if (!rtc_time_valid()) {
        s_solar_valid = false;
        step_towards(CONFIG_DASH_BRIGHTNESS_DAY_PCT);
        return;
    }

    time_t now = time(NULL);
    struct tm now_tm;
    gmtime_r(&now, &now_tm);

    struct tm midnight_tm = now_tm;
    midnight_tm.tm_hour = 0;
    midnight_tm.tm_min  = 0;
    midnight_tm.tm_sec  = 0;
    time_t today_midnight = timegm(&midnight_tm);

    if (!s_solar_valid || today_midnight != s_cached_day) {
        double lat = atof(CONFIG_DASH_LATITUDE);
        double lon = atof(CONFIG_DASH_LONGITUDE);
        if (solar_sunrise_sunset(now, lat, lon, &s_sunrise_utc, &s_sunset_utc)) {
            s_cached_day  = today_midnight;
            s_solar_valid = true;
            struct tm sr, ss;
            gmtime_r(&s_sunrise_utc, &sr);
            gmtime_r(&s_sunset_utc, &ss);
            ESP_LOGI(TAG, "sunrise=%02d:%02d sunset=%02d:%02d UTC",
                     sr.tm_hour, sr.tm_min, ss.tm_hour, ss.tm_min);
        } else {
            // Defensive only — not expected at UK latitudes. Fall back to
            // day brightness rather than fade against garbage times.
            ESP_LOGW(TAG, "sunrise/sunset undefined for this lat/long, holding day brightness");
            s_solar_valid = false;
            step_towards(CONFIG_DASH_BRIGHTNESS_DAY_PCT);
            return;
        }
    }

    step_towards(target_brightness(now));
}

void ui_brightness_start(void)
{
    lv_timer_create(brightness_tick_cb, TICK_MS, NULL);
}
