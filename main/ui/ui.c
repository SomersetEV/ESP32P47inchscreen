#include "esp_log.h"
#include "bsp/esp-bsp.h"

#include "ui.h"
#include "ui_brightness.h"

static const char *TAG = "UI";

lv_font_t *ui_font_huge;
lv_font_t *ui_font_medium;
lv_font_t *ui_font_small;

// EMBED_FILES symbols for the assets in main/assets.
extern const uint8_t ttf_start[] asm("_binary_Montserrat_Medium_ttf_start");
extern const uint8_t ttf_end[]   asm("_binary_Montserrat_Medium_ttf_end");

static lv_obj_t *s_splash;
static lv_obj_t *s_dash;

#define SPLASH_HOLD_MS     1500
#define SPLASH_FADE_OUT_MS 600
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
    lv_screen_load_anim(s_dash, LV_SCR_LOAD_ANIM_FADE_IN, SPLASH_FADE_OUT_MS, 0, true);
    s_splash = NULL;
    ESP_LOGI(TAG, "Dash screen loaded");

    // Starts only now, after the boot ramp above has already driven the
    // backlight to 100 % — so the two never race over the LEDC duty cycle.
    ui_brightness_start();
}

/*
 * Hold the backlight off before the BSP touches it at all. bsp_display_
 * brightness_init() configures the LEDC channel with output_invert = 1, so
 * "0 % brightness" is driven as a physical high at the pin — but that LEDC
 * config only takes effect once bsp_display_start_with_config() runs, and
 * the MIPI panel init after it can take a further moment to train the link.
 * In that gap GPIO32 is left floating/at its reset state, which on this
 * board reads as backlight-on, giving a brief uncontrolled flash before the
 * real 0->100% ramp — i.e. what looks like the splash fading in twice.
 * Driving the pin high as a plain GPIO first closes that gap.
 */
static void backlight_hold_off(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BSP_LCD_BACKLIGHT,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(BSP_LCD_BACKLIGHT, 1);
}

esp_err_t ui_start(void)
{
    backlight_hold_off();

    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg  = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation        = ESP_LV_ADAPTER_ROTATE_0,
        // TRIPLE_FULL draws straight into the 3 DPI panel buffers instead of
        // copying through a small partial SRAM buffer. That partial-buffer
        // path (the DEFAULT_MIPI_DSI mode) couldn't keep up with full-screen
        // LVGL animations like the splash fade, which stuttered as a result.
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_FULL,
        .touch_flags     = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
    };

    // The LVGL task stack lives in PSRAM; tiny_ttf glyph caching wants more
    // room than the 8 KB default leaves.
    cfg.lv_adapter_cfg.task_stack_size = 16 * 1024;
    cfg.lv_adapter_cfg.stack_in_psram  = true;

    // The adapter default is no core affinity, which lets LVGL drift onto
    // core 1 and breaks the core split. Pin it to core 0; the logger task
    // outranks it there (see main.c), so rendering cannot stall the drain.
    cfg.lv_adapter_cfg.task_core_id = 0;

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
    // Plain load, no LVGL opacity animation: this is the very first screen,
    // so the backlight ramp below (0 -> 100 % brightness) already gives the
    // fade-in look without doubling up on animated compositing.
    lv_screen_load(s_splash);

    int *pct = malloc(sizeof(int));
    if (pct) {
        *pct = 0;
        lv_timer_create(backlight_ramp_cb, BACKLIGHT_STEP_MS, pct);
    } else {
        bsp_display_brightness_set(100);   // no ramp, but not left dark
    }

    lv_timer_create(splash_done_cb, SPLASH_HOLD_MS, NULL);

    bsp_display_unlock();

    ESP_LOGI(TAG, "Display started, splash shown");
    return ESP_OK;
}
