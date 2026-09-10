#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "lvgl.h"

/*
 * ui.h — display bring-up, shared fonts and the screen handover.
 *
 * LVGL is not thread safe. Every lv_* call in this project sits between
 * bsp_display_lock() and bsp_display_unlock(); the helpers here are called
 * from app_main before the dash timer starts, and from inside the LVGL task
 * afterwards.
 */

// Brings up the panel and the LVGL adapter, builds both screens, and shows the
// splash with the backlight ramped up from zero. Returns ESP_OK on success.
esp_err_t ui_start(void);

// Fonts built from the embedded Montserrat TTF. Created by ui_start().
extern lv_font_t *ui_font_huge;    // RPM / MPH / kW values
extern lv_font_t *ui_font_medium;  // unit labels, clock
extern lv_font_t *ui_font_small;   // bottom grid

// Screens.
lv_obj_t *ui_splash_create(void);
lv_obj_t *ui_dash_create(void);

// Update the BLE chip in the top bar. Safe to call from any task: it takes the
// LVGL lock itself. The SD and trip chips are polled by the dash refresh timer
// and need no equivalent.
void ui_dash_set_ble(bool connected);

// Called by the splash timer when the hold expires: fades the dash in and
// starts the dash refresh timer.
void ui_show_dash(void);

// Palette shared by both screens.
#define UI_COL_BG      lv_color_hex(0x101314)
#define UI_COL_PANEL   lv_color_hex(0x1b2022)
#define UI_COL_TEXT    lv_color_hex(0xe8eceb)
#define UI_COL_DIM     lv_color_hex(0x8b9694)
#define UI_COL_ACCENT  lv_color_hex(0x56be82)
#define UI_COL_WARN    lv_color_hex(0xd8a341)
#define UI_COL_ALERT   lv_color_hex(0xd85f4a)
