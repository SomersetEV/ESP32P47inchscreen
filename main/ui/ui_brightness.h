#pragma once

/*
 * ui_brightness.h — sunrise/sunset-driven backlight.
 *
 * Runs a coarse (60 s) lv_timer that keeps the backlight at
 * CONFIG_DASH_BRIGHTNESS_DAY_PCT by day, CONFIG_DASH_BRIGHTNESS_NIGHT_PCT by
 * night, and fades linearly between them across a
 * CONFIG_DASH_BRIGHTNESS_FADE_MINUTES window centred on the real sunrise/
 * sunset for CONFIG_DASH_LATITUDE/CONFIG_DASH_LONGITUDE. Falls back to day
 * brightness whenever rtc_time_valid() is false.
 *
 * Must be started only after the boot splash ramp (backlight_ramp_cb in
 * ui.c) has finished driving the backlight, so the two never fight over the
 * LEDC duty cycle.
 */
void ui_brightness_start(void);
