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
 * The dash, in an LCARS treatment. 1024x600, laid out as:
 *
 *   top bar (110)  header elbow | status chips | clock
 *   main row (300) RPM  |  MPH  |  kW      <- the three numbers that matter
 *   bottom  (190)  2x5 grid of everything else
 *
 * Two elbows frame it: the header elbow bends from the left side bar into a
 * bar under the chips, and the body elbow bends from the side bar into a bar
 * over the main row. The side bar is split into bands that line up with the
 * sections beside them (DRIVE, SYSTEMS, and the current log session).
 *
 * Still no dials or fake gauges: large digits on a black ground. The frame is
 * built once from plain rectangles and never changes, so it costs nothing
 * after the first frame.
 *
 * A 100 ms lv_timer copies vehicle_state and writes only the labels whose text
 * or colour actually changed, which keeps the redraw cost near zero when the
 * bus is idle.
 */

#define STALE_MS      2000   // no frame for this long -> every value shows "--"
#define REFRESH_MS    100
#define BLINK_TICKS   6      // CAN STALE chip blinks with a 2 x 600 ms period

// ── Frame geometry ───────────────────────────────────────────────────────────
#define SIDE_W        150    // left side bar
#define ELBOW_R       54     // outer corner of each elbow
#define FILLET_R      24     // inner corner where the side bar meets its bar
#define CONTENT_X     166    // left edge of the main row and the grid
#define CONTENT_W     (1024 - CONTENT_X)

#define HDR_H         110
#define HDR_BAR_Y     96
#define BAR_H         14
#define BODY_Y        118
#define MID_BAR_Y     400
#define GRID_Y        414
#define CELL_W        171
#define CELL_H        88
#define ROW_PITCH     92

// ── Widgets kept for updating ────────────────────────────────────────────────
typedef struct {
    lv_obj_t *box;
    lv_obj_t *label;
} chip_t;

static lv_obj_t *s_clock, *s_date, *s_session;
static chip_t    s_chip_sd, s_chip_ble, s_chip_can, s_chip_trip;
static lv_obj_t *s_rpm, *s_mph, *s_kw;

// Bottom grid, in display order.
enum { C_BATT, C_MOTOR, C_AMPS, C_SOC, C_PACKV,
       C_INV,  C_CHG,   C_12V,  C_CELLS, C_DELTA, C_COUNT };

static const char *const s_cell_caption[C_COUNT] = {
    "BATT \xC2\xB0""C", "MOTOR \xC2\xB0""C", "AMPS", "SOC",     "PACK V",
    "INV \xC2\xB0""C",  "CHG \xC2\xB0""C",   "12V",  "CELL mV", "DELTA mV",
};

static lv_obj_t *s_cell[C_COUNT];

static lv_timer_t *s_refresh_timer;
static uint8_t     s_blink_tick;

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

static void set_bg_if_changed(lv_obj_t *o, lv_color_t c)
{
    lv_color_t cur = lv_obj_get_style_bg_color(o, LV_PART_MAIN);
    if (lv_color_to_u32(cur) != lv_color_to_u32(c)) {
        lv_obj_set_style_bg_color(o, c, 0);
    }
}

static void set_opa_if_changed(lv_obj_t *o, lv_opa_t opa)
{
    if (lv_obj_get_style_opa(o, LV_PART_MAIN) != opa) {
        lv_obj_set_style_opa(o, opa, 0);
    }
}

// Temperature colouring is shared by every temperature cell.
static lv_color_t temp_colour(int c10)
{
    if (c10 >= 800) return UI_COL_ALERT;
    if (c10 >= 600) return UI_COL_WARN;
    return UI_COL_TEXT;
}

// ── Frame primitives ─────────────────────────────────────────────────────────

// A plain filled rectangle. Everything in the frame is built from these.
static lv_obj_t *rect(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                      lv_coord_t w, lv_coord_t h, lv_color_t c, lv_coord_t r)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_color(o, c, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(o, r, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return o;
}

/*
 * LVGL has one radius for all four corners, so a shape rounded on one side is
 * a rounded rectangle with square rectangles of the same colour laid over the
 * corners that should stay square.
 */

// Bar segment with a rounded right end (the last segment of a bar).
static void bar_end(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                    lv_coord_t w, lv_coord_t h, lv_color_t c)
{
    rect(parent, x, y, w, h, c, h / 2);
    rect(parent, x, y, w / 2, h, c, 0);
}

// Black text sitting in the bottom-right corner of a frame block, LCARS style.
static lv_obj_t *frame_label(lv_obj_t *block, const char *text,
                             const lv_font_t *font, lv_coord_t inset_x,
                             lv_coord_t inset_y)
{
    lv_obj_t *l = lv_label_create(block);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, UI_COL_BG, 0);
    lv_obj_set_style_text_letter_space(l, 1, 0);
    lv_obj_align(l, LV_ALIGN_BOTTOM_RIGHT, -inset_x, -inset_y);
    return l;
}

/*
 * A pill with black text pushed to its bottom-right corner: the status chips,
 * the unit tags under the big numbers and the grid captions.
 */
static lv_obj_t *pill(lv_obj_t *parent, const char *text, const lv_font_t *font,
                      lv_color_t c, lv_coord_t h, lv_coord_t min_w,
                      lv_coord_t pad_x, lv_coord_t pad_bottom, lv_obj_t **label_out)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_remove_style_all(p);
    lv_obj_set_size(p, LV_SIZE_CONTENT, h);
    lv_obj_set_style_min_width(p, min_w, 0);
    lv_obj_set_style_radius(p, h / 2, 0);
    lv_obj_set_style_bg_color(p, c, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(p, pad_x, 0);
    lv_obj_set_style_pad_bottom(p, pad_bottom, 0);
    lv_obj_set_flex_flow(p, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(p, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_remove_flag(p, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *l = lv_label_create(p);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, UI_COL_BG, 0);
    lv_obj_set_style_text_letter_space(l, 1, 0);

    if (label_out) *label_out = l;
    return p;
}

static void make_chip(lv_obj_t *parent, chip_t *chip, const char *text,
                      lv_color_t c)
{
    chip->box = pill(parent, text, ui_font_label, c, 46, 112, 18, 4, &chip->label);
}

static void set_chip(chip_t *chip, const char *text, lv_color_t c)
{
    set_text_if_changed(chip->label, text);
    set_bg_if_changed(chip->box, c);
}

// ── Sections ─────────────────────────────────────────────────────────────────

static void build_header(lv_obj_t *scr)
{
    // Elbow: side block rounded only at its bottom-left, plus an inner fillet
    // where it meets the bar.
    rect(scr, 0, 0, SIDE_W, HDR_H / 2, UI_COL_ORANGE, 0);
    rect(scr, SIDE_W / 2, 0, SIDE_W / 2, HDR_H, UI_COL_ORANGE, 0);
    lv_obj_t *side = rect(scr, 0, 0, SIDE_W, HDR_H, UI_COL_ORANGE, ELBOW_R);

    lv_obj_t *name = lv_label_create(side);
    lv_label_set_text(name, "SOMERSET\nEV");
    lv_obj_set_style_text_font(name, ui_font_title, 0);
    lv_obj_set_style_text_color(name, UI_COL_BG, 0);
    lv_obj_set_style_text_align(name, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_letter_space(name, 1, 0);
    lv_obj_set_style_text_line_space(name, -4, 0);
    lv_obj_align(name, LV_ALIGN_TOP_RIGHT, -12, 6);

    rect(scr, SIDE_W, HDR_BAR_Y - FILLET_R, FILLET_R, FILLET_R, UI_COL_ORANGE, 0);
    rect(scr, SIDE_W, HDR_BAR_Y - 2 * FILLET_R, 2 * FILLET_R, 2 * FILLET_R,
         UI_COL_BG, LV_RADIUS_CIRCLE);

    rect(scr, SIDE_W, HDR_BAR_Y, 354, BAR_H, UI_COL_ORANGE, 0);
    rect(scr, 510,    HDR_BAR_Y,  90, BAR_H, UI_COL_LILAC,  0);
    rect(scr, 606,    HDR_BAR_Y, 240, BAR_H, UI_COL_PEACH,  0);
    bar_end(scr, 852, HDR_BAR_Y, 172, BAR_H, UI_COL_BLUE);

    // Status chips, in a flex row so a hidden TRIP chip leaves no gap.
    lv_obj_t *row = lv_obj_create(scr);
    lv_obj_remove_style_all(row);
    lv_obj_set_pos(row, 188, 22);
    lv_obj_set_size(row, 560, 46);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 10, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    make_chip(row, &s_chip_sd,   "NO SD",     UI_COL_ALERT);
    make_chip(row, &s_chip_ble,  "NO BLE",    UI_COL_IDLE);
    make_chip(row, &s_chip_can,  "CAN STALE", UI_COL_ALERT);
    make_chip(row, &s_chip_trip, "TRIP",      UI_COL_WARN);
    lv_obj_add_flag(s_chip_trip.box, LV_OBJ_FLAG_HIDDEN);

    s_clock = lv_label_create(scr);
    lv_label_set_text(s_clock, "--:--");
    lv_obj_set_style_text_font(s_clock, ui_font_clock, 0);
    lv_obj_set_style_text_color(s_clock, UI_COL_ORANGE, 0);
    lv_obj_align(s_clock, LV_ALIGN_TOP_RIGHT, -20, 0);

    s_date = lv_label_create(scr);
    lv_label_set_text(s_date, "NO CLOCK");
    lv_obj_set_style_text_font(s_date, ui_font_small, 0);
    lv_obj_set_style_text_color(s_date, UI_COL_LILAC, 0);
    lv_obj_set_style_text_letter_space(s_date, 3, 0);
    lv_obj_align(s_date, LV_ALIGN_TOP_RIGHT, -22, 68);
}

static void build_body_frame(lv_obj_t *scr)
{
    // Elbow: side block rounded only at its top-left.
    const lv_coord_t side_h = MID_BAR_Y - BODY_Y;
    rect(scr, 0, BODY_Y + side_h / 2, SIDE_W, side_h - side_h / 2, UI_COL_PEACH, 0);
    rect(scr, SIDE_W / 2, BODY_Y, SIDE_W / 2, side_h, UI_COL_PEACH, 0);
    lv_obj_t *side = rect(scr, 0, BODY_Y, SIDE_W, side_h, UI_COL_PEACH, ELBOW_R);
    frame_label(side, "DRIVE 01", ui_font_small, 12, 8);

    rect(scr, SIDE_W, BODY_Y + BAR_H, FILLET_R, FILLET_R, UI_COL_PEACH, 0);
    rect(scr, SIDE_W, BODY_Y + BAR_H, 2 * FILLET_R, 2 * FILLET_R,
         UI_COL_BG, LV_RADIUS_CIRCLE);

    rect(scr, SIDE_W, BODY_Y, 144, BAR_H, UI_COL_PEACH,  0);
    rect(scr, 300,    BODY_Y, 420, BAR_H, UI_COL_ORANGE, 0);
    rect(scr, 726,    BODY_Y,  60, BAR_H, UI_COL_ALERT,  0);
    bar_end(scr, 792, BODY_Y, 232, BAR_H, UI_COL_LILAC);

    // Side bands beside the grid.
    lv_obj_t *sys = rect(scr, 0, 406, SIDE_W, 128, UI_COL_LILAC, 0);
    frame_label(sys, "SYSTEMS 02", ui_font_small, 12, 8);

    lv_obj_t *ses = rect(scr, 0, 540, SIDE_W, 60, UI_COL_BLUE, 0);
    s_session = frame_label(ses, "SESSION ----", ui_font_small, 12, 6);

    // Divider between the main row and the grid.
    rect(scr, CONTENT_X, MID_BAR_Y, 240, 6, UI_COL_LILAC, 0);
    rect(scr, 412,       MID_BAR_Y, 500, 6, UI_COL_SEP,   0);
    bar_end(scr, 918,    MID_BAR_Y, 106, 6, UI_COL_BLUE);
}

// One big value with its unit pill in the main row.
static lv_obj_t *make_big(lv_obj_t *parent, lv_coord_t x, lv_coord_t w,
                          const char *unit, lv_color_t unit_col)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_remove_style_all(cont);
    lv_obj_set_pos(cont, x, 150);
    lv_obj_set_size(cont, w, 244);
    lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *val = lv_label_create(cont);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_font(val, ui_font_huge, 0);
    lv_obj_set_style_text_color(val, UI_COL_IDLE, 0);
    lv_obj_align(val, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *p = pill(cont, unit, ui_font_label, unit_col, 30, 0, 16, 1, NULL);
    lv_obj_align(p, LV_ALIGN_BOTTOM_MID, 0, -4);

    return val;
}

// One small labelled value in the bottom grid.
static void make_cell(lv_obj_t *parent, int idx, int col, int row)
{
    // Caption colours cycle across the columns, like the frame bands.
    static const uint32_t cap_col[5] = {
        0xc79bd6, 0xffbb88, 0x8fa8ff, 0xff9c3f, 0xc79bd6,
    };

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_remove_style_all(cont);
    lv_obj_set_pos(cont, CONTENT_X + col * CELL_W, GRID_Y + row * ROW_PITCH);
    lv_obj_set_size(cont, CELL_W, CELL_H);
    lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *cap = pill(cont, s_cell_caption[idx], ui_font_small,
                         lv_color_hex(cap_col[col]), 24, 0, 12, 1, NULL);
    lv_obj_set_pos(cap, 10, 6);

    lv_obj_t *val = lv_label_create(cont);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_font(val, ui_font_value, 0);
    lv_obj_set_style_text_color(val, UI_COL_IDLE, 0);
    lv_obj_align(val, LV_ALIGN_BOTTOM_RIGHT, -16, 0);

    s_cell[idx] = val;
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
        for (char *c = buf; *c; c++) {
            if (*c >= 'a' && *c <= 'z') *c -= 'a' - 'A';
        }
        set_text_if_changed(s_date, buf);
    } else {
        set_text_if_changed(s_clock, "--:--");
        set_text_if_changed(s_date, "NO CLOCK");
    }

    // ── Status chips ────────────────────────────────────────────────────────
    // Driven from here rather than pushed in from the logger: this timer
    // already holds the LVGL lock, and the state is only ever polled.
    const bool logging = can_logger_is_logging();
    set_chip(&s_chip_sd, logging ? "SD LOG" : "NO SD",
             logging ? UI_COL_ORANGE : UI_COL_ALERT);

    const bool trip = can_logger_trip_active();
    if (trip == lv_obj_has_flag(s_chip_trip.box, LV_OBJ_FLAG_HIDDEN)) {
        if (trip) lv_obj_remove_flag(s_chip_trip.box, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_add_flag(s_chip_trip.box, LV_OBJ_FLAG_HIDDEN);
    }

    snprintf(buf, sizeof(buf), "SESSION %04lu",
             (unsigned long)can_logger_session_id());
    set_text_if_changed(s_session, buf);

    // ── Main three ──────────────────────────────────────────────────────────
    if (fresh) {
        snprintf(buf, sizeof(buf), "%d", r->motor_rpm);
        set_text_if_changed(s_rpm, buf);
        set_colour_if_changed(s_rpm, UI_COL_TEXT);

        int mph_round = (s.speed_mph_x10 >= 0) ? (s.speed_mph_x10 + 5) / 10
                                                : -((-s.speed_mph_x10 + 5) / 10);
        snprintf(buf, sizeof(buf), "%d", mph_round);
        set_text_if_changed(s_mph, buf);
        set_colour_if_changed(s_mph, UI_COL_TEXT);

        snprintf(buf, sizeof(buf), "%.1f", s.power_w / 1000.0f);
        set_text_if_changed(s_kw, buf);
        // Regen reads blue: power flowing back into the pack.
        set_colour_if_changed(s_kw, s.power_w < 0 ? UI_COL_REGEN : UI_COL_TEXT);
    } else {
        set_text_if_changed(s_rpm, "--");
        set_text_if_changed(s_mph, "--");
        set_text_if_changed(s_kw,  "--");
        set_colour_if_changed(s_rpm, UI_COL_IDLE);
        set_colour_if_changed(s_mph, UI_COL_IDLE);
        set_colour_if_changed(s_kw,  UI_COL_IDLE);
    }

    // ── Bottom grid ─────────────────────────────────────────────────────────
    if (!fresh) {
        for (int i = 0; i < C_COUNT; i++) {
            set_text_if_changed(s_cell[i], "--");
            set_colour_if_changed(s_cell[i], UI_COL_IDLE);
        }
        set_chip(&s_chip_can, "CAN STALE", UI_COL_ALERT);
        // Blink by opacity: only the chip's own area is redrawn.
        s_blink_tick = (uint8_t)((s_blink_tick + 1) % (2 * BLINK_TICKS));
        set_opa_if_changed(s_chip_can.box,
                           s_blink_tick < BLINK_TICKS ? LV_OPA_COVER : LV_OPA_40);
        return;
    }

    s_blink_tick = 0;
    set_opa_if_changed(s_chip_can.box, LV_OPA_COVER);
    set_chip(&s_chip_can, "CAN OK", UI_COL_ORANGE);

    snprintf(buf, sizeof(buf), "%u%%", r->soc);
    set_text_if_changed(s_cell[C_SOC], buf);
    set_colour_if_changed(s_cell[C_SOC], r->soc < 15 ? UI_COL_ALERT : UI_COL_TEXT);

    snprintf(buf, sizeof(buf), "%.1f", r->pack_voltage_mv / 1000.0f);
    set_text_if_changed(s_cell[C_PACKV], buf);
    set_colour_if_changed(s_cell[C_PACKV], UI_COL_TEXT);

    snprintf(buf, sizeof(buf), "%.1f", r->pack_current_ma / 1000.0f);
    set_text_if_changed(s_cell[C_AMPS], buf);
    set_colour_if_changed(s_cell[C_AMPS],
                          r->pack_current_ma < 0 ? UI_COL_REGEN : UI_COL_TEXT);

    snprintf(buf, sizeof(buf), "%.1f", r->bms_temp_max / 10.0f);
    set_text_if_changed(s_cell[C_BATT], buf);
    set_colour_if_changed(s_cell[C_BATT], temp_colour(r->bms_temp_max));

    snprintf(buf, sizeof(buf), "%.1f", r->motor_temp / 10.0f);
    set_text_if_changed(s_cell[C_MOTOR], buf);
    set_colour_if_changed(s_cell[C_MOTOR], temp_colour(r->motor_temp));

    snprintf(buf, sizeof(buf), "%.1f", r->inverter_temp / 10.0f);
    set_text_if_changed(s_cell[C_INV], buf);
    set_colour_if_changed(s_cell[C_INV], temp_colour(r->inverter_temp));

    snprintf(buf, sizeof(buf), "%.1f", r->charger_temp / 10.0f);
    set_text_if_changed(s_cell[C_CHG], buf);
    set_colour_if_changed(s_cell[C_CHG], temp_colour(r->charger_temp));

    snprintf(buf, sizeof(buf), "%.1f", r->lv_volts_mv / 1000.0f);
    set_text_if_changed(s_cell[C_12V], buf);
    set_colour_if_changed(s_cell[C_12V], UI_COL_TEXT);

    snprintf(buf, sizeof(buf), "%u/%u", r->cell_voltage_min, r->cell_voltage_max);
    set_text_if_changed(s_cell[C_CELLS], buf);
    set_colour_if_changed(s_cell[C_CELLS], UI_COL_TEXT);

    snprintf(buf, sizeof(buf), "%u", s.cell_delta_mv);
    set_text_if_changed(s_cell[C_DELTA], buf);
    set_colour_if_changed(s_cell[C_DELTA], UI_COL_TEXT);
}

/*
 * BLE connection state is pushed in, unlike the SD and trip chips: it lives in
 * the BLE task and is event-driven rather than pollable, so there is nothing
 * for the refresh timer to read.
 */
void ui_dash_set_ble(bool connected)
{
    if (!s_chip_ble.box) return;
    bsp_display_lock(0);
    set_chip(&s_chip_ble, connected ? "BLE" : "NO BLE",
             connected ? UI_COL_BLUE : UI_COL_IDLE);
    bsp_display_unlock();
}

lv_obj_t *ui_dash_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, UI_COL_BG, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    build_header(scr);
    build_body_frame(scr);

    // ── Main row ────────────────────────────────────────────────────────────
    // Same proportions as before (400 / 254 / 370), scaled into the width
    // left of the side bar. RPM needs four digits, MPH two, and kW up to
    // five characters ("-50.0").
    s_rpm = make_big(scr, CONTENT_X, 335, "RPM", UI_COL_ORANGE);
    rect(scr, 500, 176, 4, 196, UI_COL_SEP, 0);
    s_mph = make_big(scr, 503,       211, "MPH", UI_COL_LILAC);
    rect(scr, 713, 176, 4, 196, UI_COL_SEP, 0);
    s_kw  = make_big(scr, 716,       308, "kW",  UI_COL_BLUE);

    // ── Bottom grid ─────────────────────────────────────────────────────────
    for (int i = 0; i < C_COUNT; i++) {
        make_cell(scr, i, i % 5, i / 5);
    }

    s_refresh_timer = lv_timer_create(refresh_cb, REFRESH_MS, NULL);

    return scr;
}
