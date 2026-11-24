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

// Fonts built from the embedded Antonio SemiBold TTF (a narrow face, which is
// what lets four RPM digits fit at 150 px). Created by ui_start().
extern lv_font_t *ui_font_huge;    // RPM / MPH / kW values (150)
extern lv_font_t *ui_font_clock;   // clock (58)
extern lv_font_t *ui_font_title;   // name in the header elbow (30)
extern lv_font_t *ui_font_value;   // bottom grid values (42)
extern lv_font_t *ui_font_label;   // status chips, unit pills (22)
extern lv_font_t *ui_font_small;   // grid captions, side-bar labels, date (17)

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

// LCARS palette shared by both screens. Frame colours carry no meaning of
// their own; WARN / ALERT / REGEN / IDLE are the state colours.
#define UI_COL_BG       lv_color_hex(0x000000)
#define UI_COL_ORANGE   lv_color_hex(0xff9c3f)   // primary frame, "ok" chips
#define UI_COL_PEACH    lv_color_hex(0xffbb88)
#define UI_COL_LILAC    lv_color_hex(0xc79bd6)
#define UI_COL_BLUE     lv_color_hex(0x8fa8ff)   // frame, BLE chip
#define UI_COL_SEP      lv_color_hex(0x26222f)   // column separators, idle bar
#define UI_COL_TEXT     lv_color_hex(0xffe2c2)   // numeric readouts
#define UI_COL_IDLE     lv_color_hex(0x5f5a78)   // inactive chip, stale values
#define UI_COL_REGEN    UI_COL_BLUE              // negative kW / amps
#define UI_COL_WARN     lv_color_hex(0xffcc55)   // >= 60 C, trip chip
#define UI_COL_ALERT    lv_color_hex(0xff4f3c)   // >= 80 C, low SOC, faults
