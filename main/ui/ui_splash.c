#include "board.h"
#include "ui.h"

/*
 * Splash screen: logo, name and firmware version on black, shown while the
 * rest of the system starts. ui.c ramps the backlight up over it and fades to
 * the dash after a hold.
 */

extern const uint8_t logo_start[] asm("_binary_logo_png_start");
extern const uint8_t logo_end[]   asm("_binary_logo_png_end");

// LODEPNG decodes this at runtime, so the descriptor just points at the raw
// file bytes. Replacing main/assets/logo.png is all the real logo needs.
static lv_image_dsc_t s_logo;

lv_obj_t *ui_splash_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, UI_COL_BG, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    s_logo.header.cf        = LV_COLOR_FORMAT_RAW;
    s_logo.header.w         = 200;
    s_logo.header.h         = 100;
    s_logo.data             = logo_start;
    s_logo.data_size        = (uint32_t)(logo_end - logo_start);

    lv_obj_t *img = lv_image_create(scr);
    lv_image_set_src(img, &s_logo);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, -70);

    lv_obj_t *name = lv_label_create(scr);
    lv_label_set_text(name, "Somerset EV");
    lv_obj_set_style_text_font(name, ui_font_medium, 0);
    lv_obj_set_style_text_color(name, UI_COL_TEXT, 0);
    lv_obj_align(name, LV_ALIGN_CENTER, 0, 40);

    lv_obj_t *ver = lv_label_create(scr);
    lv_label_set_text(ver, "Tractor Dash  v" FW_VERSION);
    lv_obj_set_style_text_font(ver, ui_font_small, 0);
    lv_obj_set_style_text_color(ver, UI_COL_DIM, 0);
    lv_obj_align(ver, LV_ALIGN_CENTER, 0, 90);

    return scr;
}
