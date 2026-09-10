#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "bsp/esp-bsp.h"

#include "board.h"
#include "ui.h"
#include "vehicle_state.h"
#include "can_logger.h"

/*
 * The dash. 1024x600, laid out as:
 *
 *   top bar (110)  logo | status chips | clock
 *   main row (300) RPM  |  MPH  |  kW      <- the three numbers that matter
 *   bottom  (190)  2x5 grid of everything else
 *
 * No dials or fake gauges: the tractor is read at a glance in daylight, so it
 * is large digits on a dark ground.
 *
 * A 100 ms lv_timer copies vehicle_state and writes only the labels whose text
 * actually changed, which keeps the redraw cost near zero when the bus is idle.
 */

#define STALE_MS      2000   // no frame for this long -> every value shows "--"
#define REFRESH_MS    100

extern const uint8_t logo_start[] asm("_binary_logo_png_start");
extern const uint8_t logo_end[]   asm("_binary_logo_png_end");
static lv_image_dsc_t s_logo;

// ── Widgets kept for updating ────────────────────────────────────────────────
static lv_obj_t *s_clock, *s_date;
static lv_obj_t *s_chip_sd, *s_chip_ble, *s_chip_can, *s_chip_trip;
static lv_obj_t *s_rpm, *s_mph, *s_kw;

typedef struct {
    lv_obj_t   *value;
    const char *label;
} cell_t;

// Bottom grid, in display order.
enum { C_SOC, C_PACKV, C_AMPS, C_BATT, C_MOTOR,
       C_INV, C_CHG, C_12V, C_CELLS, C_DELTA, C_COUNT };

static cell_t s_cell[C_COUNT];

static lv_timer_t *s_refresh_timer;

// Only touch the label when the text really changed — lv_label_set_text
// invalidates the area every time otherwise.
static void set_text_if_changed(lv_obj_t *label, const char *txt)
{
    const char *cur = lv_label_get_text(label);
    if (!cur || strcmp(cur, txt) != 0) {
        lv_label_set_text(label, txt);
    }
}

static void set_colour_if_changed(lv_obj_t *o, lv_color_t c)
{
    lv_color_t cur = lv_obj_get_style_text_color(o, LV_PART_MAIN);
    if (lv_color_to_u32(cur) != lv_color_to_u32(c)) {
        lv_obj_set_style_text_color(o, c, 0);
    }
}

// Temperature colouring is shared by every temperature cell.
static lv_color_t temp_colour(int c10)
{
    if (c10 >= 800) return UI_COL_ALERT;
    if (c10 >= 600) return UI_COL_WARN;
    return UI_COL_TEXT;
}

static lv_obj_t *make_chip(lv_obj_t *parent, const char *text, lv_coord_t x)
{
    lv_obj_t *chip = lv_label_create(parent);
    lv_label_set_text(chip, text);
    lv_obj_set_style_text_font(chip, ui_font_small, 0);
    lv_obj_set_style_text_color(chip, UI_COL_DIM, 0);
    lv_obj_align(chip, LV_ALIGN_LEFT_MID, x, 0);
    return chip;
}

// One big value + unit in the main row.
static lv_obj_t *make_big(lv_obj_t *parent, lv_coord_t x, lv_coord_t w,
                          const char *unit)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_remove_style_all(cont);
    lv_obj_set_size(cont, w, 300);
    lv_obj_set_pos(cont, x, 110);
    lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *val = lv_label_create(cont);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_font(val, ui_font_huge, 0);
    lv_obj_set_style_text_color(val, UI_COL_TEXT, 0);
    lv_obj_align(val, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *lbl = lv_label_create(cont);
    lv_label_set_text(lbl, unit);
    lv_obj_set_style_text_font(lbl, ui_font_medium, 0);
    lv_obj_set_style_text_color(lbl, UI_COL_DIM, 0);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -10);

    return val;
}

// One small labelled value in the bottom grid.
static void make_cell(lv_obj_t *parent, int idx, const char *label,
                      int col, int row)
{
    const lv_coord_t cw = 1024 / 5;
    const lv_coord_t ch = 95;

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_remove_style_all(cont);
    lv_obj_set_size(cont, cw, ch);
    lv_obj_set_pos(cont, col * cw, 410 + row * ch);
    lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *cap = lv_label_create(cont);
    lv_label_set_text(cap, label);
    lv_obj_set_style_text_font(cap, ui_font_small, 0);
    lv_obj_set_style_text_color(cap, UI_COL_DIM, 0);
    lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, 4);

    lv_obj_t *val = lv_label_create(cont);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_font(val, ui_font_medium, 0);
    lv_obj_set_style_text_color(val, UI_COL_TEXT, 0);
    lv_obj_align(val, LV_ALIGN_BOTTOM_MID, 0, -6);

    s_cell[idx].value = val;
    s_cell[idx].label = label;
}

static void refresh_cb(lv_timer_t *t)
{
    (void)t;
    vehicle_state_t s;
    vehicle_state_snapshot(&s);
    const log_record_t *r = &s.latest;

    const bool fresh = vehicle_state_is_fresh(STALE_MS);
    char buf[32];

    // ── Clock ───────────────────────────────────────────────────────────────
    time_t now = time(NULL);
    if (now > 1700000000) {          // time has been set (RTC or phone)
        struct tm tm_local;
        localtime_r(&now, &tm_local);
        strftime(buf, sizeof(buf), "%H:%M", &tm_local);
        set_text_if_changed(s_clock, buf);
        strftime(buf, sizeof(buf), "%a %d %b", &tm_local);
        set_text_if_changed(s_date, buf);
    } else {
        set_text_if_changed(s_clock, "--:--");
        set_text_if_changed(s_date, "no clock");
    }

    // ── Status chips ────────────────────────────────────────────────────────
    // Driven from here rather than pushed in from the logger: this timer
    // already holds the LVGL lock, and the state is only ever polled.
    const bool logging = can_logger_is_logging();
    set_text_if_changed(s_chip_sd, logging ? "SD LOG" : "NO SD");
    set_colour_if_changed(s_chip_sd, logging ? UI_COL_ACCENT : UI_COL_ALERT);

    const bool trip = can_logger_trip_active();
    set_text_if_changed(s_chip_trip, trip ? "TRIP" : "");
    set_colour_if_changed(s_chip_trip, UI_COL_WARN);

    // ── Main three ──────────────────────────────────────────────────────────
    if (fresh) {
        snprintf(buf, sizeof(buf), "%d", r->motor_rpm);
        set_text_if_changed(s_rpm, buf);

        snprintf(buf, sizeof(buf), "%.1f", s.speed_mph_x10 / 10.0f);
        set_text_if_changed(s_mph, buf);

        snprintf(buf, sizeof(buf), "%.1f", s.power_w / 1000.0f);
        set_text_if_changed(s_kw, buf);
        // Regen reads green: power flowing back into the pack.
        set_colour_if_changed(s_kw, s.power_w < 0 ? UI_COL_ACCENT : UI_COL_TEXT);
    } else {
        set_text_if_changed(s_rpm, "--");
        set_text_if_changed(s_mph, "--");
        set_text_if_changed(s_kw,  "--");
        set_colour_if_changed(s_kw, UI_COL_TEXT);
    }

    // ── Bottom grid ─────────────────────────────────────────────────────────
    if (!fresh) {
        for (int i = 0; i < C_COUNT; i++) {
            set_text_if_changed(s_cell[i].value, "--");
            set_colour_if_changed(s_cell[i].value, UI_COL_TEXT);
        }
        set_text_if_changed(s_chip_can, "CAN STALE");
        set_colour_if_changed(s_chip_can, UI_COL_ALERT);
        return;
    }

    set_text_if_changed(s_chip_can, "CAN OK");
    set_colour_if_changed(s_chip_can, UI_COL_ACCENT);

    snprintf(buf, sizeof(buf), "%u%%", r->soc);
    set_text_if_changed(s_cell[C_SOC].value, buf);
    set_colour_if_changed(s_cell[C_SOC].value,
                          r->soc < 15 ? UI_COL_ALERT : UI_COL_TEXT);

    snprintf(buf, sizeof(buf), "%.1f", r->pack_voltage_mv / 1000.0f);
    set_text_if_changed(s_cell[C_PACKV].value, buf);

    snprintf(buf, sizeof(buf), "%.1f", r->pack_current_ma / 1000.0f);
    set_text_if_changed(s_cell[C_AMPS].value, buf);

    snprintf(buf, sizeof(buf), "%.1f", r->bms_temp_max / 10.0f);
    set_text_if_changed(s_cell[C_BATT].value, buf);
    set_colour_if_changed(s_cell[C_BATT].value, temp_colour(r->bms_temp_max));

    snprintf(buf, sizeof(buf), "%.1f", r->motor_temp / 10.0f);
    set_text_if_changed(s_cell[C_MOTOR].value, buf);
    set_colour_if_changed(s_cell[C_MOTOR].value, temp_colour(r->motor_temp));

    snprintf(buf, sizeof(buf), "%.1f", r->inverter_temp / 10.0f);
    set_text_if_changed(s_cell[C_INV].value, buf);
    set_colour_if_changed(s_cell[C_INV].value, temp_colour(r->inverter_temp));

    snprintf(buf, sizeof(buf), "%.1f", r->charger_temp / 10.0f);
    set_text_if_changed(s_cell[C_CHG].value, buf);
    set_colour_if_changed(s_cell[C_CHG].value, temp_colour(r->charger_temp));

    snprintf(buf, sizeof(buf), "%.1f", r->lv_volts_mv / 1000.0f);
    set_text_if_changed(s_cell[C_12V].value, buf);

    snprintf(buf, sizeof(buf), "%u/%u", r->cell_voltage_min, r->cell_voltage_max);
    set_text_if_changed(s_cell[C_CELLS].value, buf);

    snprintf(buf, sizeof(buf), "%u", s.cell_delta_mv);
    set_text_if_changed(s_cell[C_DELTA].value, buf);
}

/*
 * BLE connection state is pushed in, unlike the SD and trip chips: it lives in
 * the BLE task and is event-driven rather than pollable, so there is nothing
 * for the refresh timer to read.
 */
void ui_dash_set_ble(bool connected)
{
    if (!s_chip_ble) return;
    bsp_display_lock(0);
    set_text_if_changed(s_chip_ble, connected ? "BLE" : "---");
    set_colour_if_changed(s_chip_ble, connected ? UI_COL_ACCENT : UI_COL_DIM);
    bsp_display_unlock();
}

lv_obj_t *ui_dash_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, UI_COL_BG, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Top bar ─────────────────────────────────────────────────────────────
    lv_obj_t *top = lv_obj_create(scr);
    lv_obj_remove_style_all(top);
    lv_obj_set_size(top, 1024, 110);
    lv_obj_set_pos(top, 0, 0);
    lv_obj_set_style_bg_color(top, UI_COL_PANEL, 0);
    lv_obj_set_style_bg_opa(top, LV_OPA_COVER, 0);
    lv_obj_remove_flag(top, LV_OBJ_FLAG_SCROLLABLE);

    s_logo.header.cf = LV_COLOR_FORMAT_RAW;
    s_logo.header.w  = 200;
    s_logo.header.h  = 100;
    s_logo.data      = logo_start;
    s_logo.data_size = (uint32_t)(logo_end - logo_start);

    lv_obj_t *img = lv_image_create(top);
    lv_image_set_src(img, &s_logo);
    lv_obj_align(img, LV_ALIGN_LEFT_MID, 16, 0);

    s_chip_sd   = make_chip(top, "NO SD",     260);
    s_chip_ble  = make_chip(top, "---",       400);
    s_chip_can  = make_chip(top, "CAN STALE", 500);
    s_chip_trip = make_chip(top, "",          660);

    s_clock = lv_label_create(top);
    lv_label_set_text(s_clock, "--:--");
    lv_obj_set_style_text_font(s_clock, ui_font_medium, 0);
    lv_obj_set_style_text_color(s_clock, UI_COL_TEXT, 0);
    lv_obj_align(s_clock, LV_ALIGN_RIGHT_MID, -16, -16);

    s_date = lv_label_create(top);
    lv_label_set_text(s_date, "no clock");
    lv_obj_set_style_text_font(s_date, ui_font_small, 0);
    lv_obj_set_style_text_color(s_date, UI_COL_DIM, 0);
    lv_obj_align(s_date, LV_ALIGN_RIGHT_MID, -16, 22);

    // ── Main row ────────────────────────────────────────────────────────────
    s_rpm = make_big(scr, 0,   341, "RPM");
    s_mph = make_big(scr, 341, 342, "MPH");
    s_kw  = make_big(scr, 683, 341, "kW");

    // ── Bottom grid ─────────────────────────────────────────────────────────
    make_cell(scr, C_SOC,   "SOC",        0, 0);
    make_cell(scr, C_PACKV, "PACK V",     1, 0);
    make_cell(scr, C_AMPS,  "AMPS",       2, 0);
    make_cell(scr, C_BATT,  "BATT \xC2\xB0""C",  3, 0);
    make_cell(scr, C_MOTOR, "MOTOR \xC2\xB0""C", 4, 0);
    make_cell(scr, C_INV,   "INV \xC2\xB0""C",   0, 1);
    make_cell(scr, C_CHG,   "CHG \xC2\xB0""C",   1, 1);
    make_cell(scr, C_12V,   "12V",        2, 1);
    make_cell(scr, C_CELLS, "CELL mV",    3, 1);
    make_cell(scr, C_DELTA, "DELTA mV",   4, 1);

    s_refresh_timer = lv_timer_create(refresh_cb, REFRESH_MS, NULL);

    return scr;
}
