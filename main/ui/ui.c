#include "esp_log.h"
#include "bsp/esp-bsp.h"

#include "ui.h"

static const char *TAG = "UI";

lv_font_t *ui_font_huge;
lv_font_t *ui_font_medium;
lv_font_t *ui_font_small;

// EMBED_FILES symbols for the assets in main/assets.
extern const uint8_t ttf_start[] asm("_binary_Montserrat_Medium_ttf_start");
extern const uint8_t ttf_end[]   asm("_binary_Montserrat_Medium_ttf_end");

static lv_obj_t *s_splash;
static lv_obj_t *s_dash;

#define SPLASH_HOLD_MS   1500
#define BACKLIGHT_STEPS  20
#define BACKLIGHT_STEP_MS 15

/*
 * Ramp the backlight from 0 to 100 % instead of switching it on. The panel
 * shows a white flash for a frame or two as the MIPI link starts, and ramping
 * hides it.
 */
static void backlight_ramp_cb(lv_timer_t *t)
{
    int *pct = (int *)lv_timer_get_user_data(t);
    *pct += 100 / BACKLIGHT_STEPS;

    if (*pct >= 100) {
        bsp_display_brightness_set(100);
        lv_timer_delete(t);
        free(pct);
        return;
    }
    bsp_display_brightness_set(*pct);
}

static void splash_done_cb(lv_timer_t *t)
{
    lv_timer_delete(t);
    ui_show_dash();
}

void ui_show_dash(void)
{
    // Deletes the splash once the animation finishes (final argument).
    lv_screen_load_anim(s_dash, LV_SCR_LOAD_ANIM_FADE_IN, 600, 0, true);
    s_splash = NULL;
    ESP_LOGI(TAG, "Dash screen loaded");
}

esp_err_t ui_start(void)
{
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg  = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation        = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_DEFAULT_MIPI_DSI,
        .touch_flags     = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
    };

    // The LVGL task stack lives in PSRAM; tiny_ttf glyph caching wants more
    // room than the 8 KB default leaves.
    cfg.lv_adapter_cfg.task_stack_size = 16 * 1024;
    cfg.lv_adapter_cfg.stack_in_psram  = true;

    lv_display_t *disp = bsp_display_start_with_config(&cfg);
    if (!disp) {
        ESP_LOGE(TAG, "bsp_display_start_with_config failed");
        return ESP_FAIL;
    }

    // Keep it dark until the splash is drawn, then ramp.
    bsp_display_brightness_set(0);

    bsp_display_lock(0);

    const size_t ttf_len = (size_t)(ttf_end - ttf_start);
    ui_font_huge   = lv_tiny_ttf_create_data(ttf_start, ttf_len, 150);
    ui_font_medium = lv_tiny_ttf_create_data(ttf_start, ttf_len, 34);
    ui_font_small  = lv_tiny_ttf_create_data(ttf_start, ttf_len, 26);

    if (!ui_font_huge || !ui_font_medium || !ui_font_small) {
        // Fall back to the built-in bitmap font rather than crashing; the dash
        // is still readable, just not as large.
        ESP_LOGW(TAG, "tiny_ttf font creation failed, falling back to Montserrat 28");
        ui_font_huge   = (lv_font_t *)&lv_font_montserrat_28;
        ui_font_medium = (lv_font_t *)&lv_font_montserrat_28;
        ui_font_small  = (lv_font_t *)&lv_font_montserrat_28;
    }

    s_dash   = ui_dash_create();
    s_splash = ui_splash_create();
    lv_screen_load(s_splash);

    int *pct = malloc(sizeof(int));
    *pct = 0;
    lv_timer_create(backlight_ramp_cb, BACKLIGHT_STEP_MS, pct);
    lv_timer_create(splash_done_cb, SPLASH_HOLD_MS, NULL);

    bsp_display_unlock();

    ESP_LOGI(TAG, "Display started, splash shown");
    return ESP_OK;
}
