#include "board.h"
#include "ui.h"

/*
 * Splash screen: small logo centered on the dash's own background color,
 * shown while the rest of the system starts. ui.c ramps the backlight up
 * over it and fades to the dash after a hold.
 *
 * Kept small deliberately: a full-screen bitmap made every animation frame
 * blend all 1024x600 pixels, which was too much per-frame work for a smooth
 * fade. A small logo over a flat color fill is cheap to composite regardless
 * of draw-buffer size or tear-avoid mode.
 */

extern const uint8_t splash_logo_start[] asm("_binary_splash_logo_png_start");
extern const uint8_t splash_logo_end[]   asm("_binary_splash_logo_png_end");

// LODEPNG decodes this at runtime, so the descriptor just points at the raw
// file bytes. Replacing main/assets/splash_logo.png is all the real logo
// needs, as long as the w/h below are kept in sync with the PNG's own size.
#define SPLASH_LOGO_W 420
#define SPLASH_LOGO_H 251
static lv_image_dsc_t s_logo;

lv_obj_t *ui_splash_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, UI_COL_BG, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    s_logo.header.cf        = LV_COLOR_FORMAT_RAW;
    s_logo.header.w         = SPLASH_LOGO_W;
    s_logo.header.h         = SPLASH_LOGO_H;
    s_logo.data             = splash_logo_start;
    s_logo.data_size        = (uint32_t)(splash_logo_end - splash_logo_start);

    lv_obj_t *img = lv_image_create(scr);
    lv_image_set_src(img, &s_logo);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    return scr;
}
