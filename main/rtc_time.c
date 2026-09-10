#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "esp_log.h"
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "pcf85063a.h"

#include "board.h"
#include "rtc_time.h"

static const char *TAG = "RTC";

#ifndef CONFIG_DASH_TZ
#define CONFIG_DASH_TZ "GMT0BST,M3.5.0/1,M10.5.0"
#endif

static pcf85063a_dev_t s_dev;
static bool s_present;
static bool s_valid;

// Anything before this is a cold RTC that has never been set.
#define PLAUSIBLE_YEAR 2024

bool rtc_time_valid(void)
{
    return s_valid;
}

esp_err_t rtc_time_init(void)
{
    // Logs stay UTC; this only affects the clock drawn on the dash.
    setenv("TZ", CONFIG_DASH_TZ, 1);
    tzset();

    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        ESP_LOGW(TAG, "I2C bus not initialised, skipping RTC");
        return ESP_ERR_INVALID_STATE;
    }

    // The part number is inferred from Waveshare's own component rather than
    // printed in the docs, so probe before trusting it.
    if (i2c_master_probe(bus, RTC_I2C_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "not found at 0x%02X - clock unset, logs will have no dates",
                 RTC_I2C_ADDR);

        // The address is inferred, so list what is actually on the bus once.
        // This is a report, not a search: nothing here changes behaviour, it
        // just saves reaching for a separate i2c_tools build.
        char found[96];
        int n = 0;
        for (uint8_t a = 0x08; a < 0x78 && n < (int)sizeof(found) - 6; a++) {
            if (i2c_master_probe(bus, a, 20) == ESP_OK) {
                n += snprintf(found + n, sizeof(found) - n, "0x%02X ", a);
            }
        }
        ESP_LOGW(TAG, "I2C devices present: %s", n ? found : "(none)");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = pcf85063a_init(&s_dev, bus, RTC_I2C_ADDR);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init failed: %s", esp_err_to_name(err));
        return err;
    }
    s_present = true;

    pcf85063a_datetime_t dt;
    err = pcf85063a_get_time_date(&s_dev, &dt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        return err;
    }

    if (dt.year < PLAUSIBLE_YEAR) {
        ESP_LOGW(TAG, "PCF85063A at 0x%02X, time not set (year %u)",
                 RTC_I2C_ADDR, dt.year);
        return ESP_OK;      // present but never set; still a pass
    }

    struct tm tm_utc = {
        .tm_year = dt.year - 1900,
        .tm_mon  = dt.month - 1,
        .tm_mday = dt.day,
        .tm_hour = dt.hour,
        .tm_min  = dt.min,
        .tm_sec  = dt.sec,
        .tm_isdst = 0,
    };

    // The RTC holds UTC, so convert with timegm rather than mktime (which
    // would apply the timezone we just set and shift the clock in summer).
    time_t epoch = timegm(&tm_utc);
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_valid = true;

    ESP_LOGI(TAG, "PCF85063A at 0x%02X, time=%04u-%02u-%02u %02u:%02u:%02u UTC",
             RTC_I2C_ADDR, dt.year, dt.month, dt.day, dt.hour, dt.min, dt.sec);
    return ESP_OK;
}

esp_err_t rtc_time_set(time_t unix_epoch)
{
    struct timeval tv = { .tv_sec = unix_epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_valid = true;

    if (!s_present) {
        ESP_LOGW(TAG, "system clock set, but no RTC to hold it across power-off");
        return ESP_ERR_NOT_FOUND;
    }

    struct tm tm_utc;
    gmtime_r(&unix_epoch, &tm_utc);

    pcf85063a_datetime_t dt = {
        .year  = (uint16_t)(tm_utc.tm_year + 1900),
        .month = (uint8_t)(tm_utc.tm_mon + 1),
        .day   = (uint8_t)tm_utc.tm_mday,
        .dotw  = (uint8_t)tm_utc.tm_wday,
        .hour  = (uint8_t)tm_utc.tm_hour,
        .min   = (uint8_t)tm_utc.tm_min,
        .sec   = (uint8_t)tm_utc.tm_sec,
    };

    esp_err_t err = pcf85063a_set_time_date(&s_dev, dt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "set to %04u-%02u-%02u %02u:%02u:%02u UTC",
                 dt.year, dt.month, dt.day, dt.hour, dt.min, dt.sec);
    }
    return err;
}
