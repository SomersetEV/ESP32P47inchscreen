#pragma once
#include "driver/gpio.h"

#define FW_VERSION "0.1.0"

/*
 * board.h — Waveshare ESP32-P4-WIFI6-Touch-LCD-7B pin assignments
 *
 * Single source of truth for every GPIO and I2C address the firmware touches.
 * Most peripherals are brought up by the Waveshare BSP; the values here are
 * either used directly (CAN) or documented so the wiring is checkable in one
 * place.
 */

// ── CAN / TWAI (onboard TJA1051, PH2.0 header) ───────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_22
#define CAN_RX_PIN  GPIO_NUM_21

// ── I2C bus 0 (BSP-owned): GT911 touch, ES8311, ES7210, RTC ──────────────────
// SDA GPIO7, SCL GPIO8 @ 400 kHz — configured by bsp_i2c_init().
#define RTC_I2C_ADDR        0x51    // PCF85063A (inferred; probed at boot)
#define TOUCH_I2C_ADDR      0x5D    // GT911 primary (0x14 backup)

// ── Display (BSP-owned) ──────────────────────────────────────────────────────
// 1024x600 MIPI-DSI EK79007, 2 lanes. Backlight GPIO32 (LEDC), reset GPIO33.
#define LCD_H_RES   1024
#define LCD_V_RES   600

// ── MicroSD (BSP-owned, SDMMC slot 0, 4-bit) ─────────────────────────────────
// CLK 43, CMD 44, D0-D3 39/40/41/42, powered by on-chip LDO channel 4.
#define MOUNT_POINT "/sdcard"
