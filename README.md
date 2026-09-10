# SomersetEV Tractor Dash

A 7" touch dash and CAN logger for an electric Leyland 255, on the Waveshare
ESP32-P4-WIFI6-Touch-LCD-7B.

It replaces the Arduino Due + Nextion dash and absorbs the LilyGo T-CAN485
logger, so one box now:

1. decodes the same 500 kbps CAN messages and shows RPM, speed and kW large;
2. logs every CAN frame to SD, expecting power to be cut without warning;
3. syncs those logs to the existing Flutter phone app over BLE;
4. timestamps logs from a battery-backed RTC.

## Build and flash

ESP-IDF v6.0.1, from PowerShell:

```powershell
& "C:\esp\v6.0.1\esp-idf\export.ps1" | Out-Null
idf.py set-target esp32p4      # first time only
idf.py build
idf.py -p COM5 flash
```

`idf.py monitor` never returns, which makes it awkward to script. To read the
boot log instead:

```powershell
powershell -File tools/capture_log.ps1 -Port COM5 -Seconds 25
```

The script opens the port, pulses RTS to reset the board so the boot log is not
missed, listens for `-Seconds`, then writes and prints `build/serial.log`. Pass
`-NoReset` to listen to a board that is already running.

`sdkconfig` is generated from `sdkconfig.defaults` and is not committed. After
changing `sdkconfig.defaults`, delete `sdkconfig` and rebuild or the change will
not take effect.

## Hardware notes

| Item | Value |
|---|---|
| Display | 1024x600 MIPI-DSI, EK79007, 2 lanes |
| Touch | GT911 @ 0x5D on I2C (SDA 7, SCL 8) |
| Backlight | GPIO32 (LEDC) |
| CAN | TWAI RX GPIO21, TX GPIO22, onboard TJA1051, PH2.0 header |
| SD | SDMMC slot 0, 4-bit (CLK 43, CMD 44, D0-D3 39/40/41/42) |
| RTC | PCF85063A @ 0x51, probed at boot |
| Wi-Fi/BLE | ESP32-C6 over SDIO via esp_hosted |
| Flash / PSRAM | 32 MB / 32 MB hex @ 200 MHz |

### Chip revision

The board in hand is **ESP32-P4 revision v1.3**, not the rev 3.x that
Waveshare's own defaults assume. IDF treats P4 rev <3.0 and >=3.0 as mutually
exclusive builds, so `sdkconfig.defaults` sets:

```
CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y
CONFIG_ESP32P4_REV_MIN_100=y
```

Without the first line the rev-min choice does not even offer v1.0, silently
falls back to v3.1, and esptool then refuses to flash. If a later board is rev
3.x, drop both lines.

### esp_hosted component requirements

`esp_hosted` (checked on both 1.4.x and 2.0.17) includes `driver/sdmmc_host.h`,
`esp_dma_utils.h` and `driver/gpio.h` without listing the IDF components that
provide them. All of those headers exist in IDF 6.0.1; they are just not on that
component's include path. The top-level `CMakeLists.txt` adds them to the common
requirements, which fixes the build without patching `managed_components/`.

### ESP32-C6 firmware

BLE runs as a NimBLE host on the P4 with the controller on the ESP32-C6, linked
by esp_hosted over SDIO. Host and slave must be on the same major line. This
project uses host **2.0.17**; if the C6 still carries 1.4-era slave firmware the
handshake will fail and BLE will not advertise. The fix is to reflash the C6
through its UART header with the matching esp_hosted `slave` example
(target esp32c6, SDIO, `CONFIG_BT_ENABLED=y`, `CONFIG_BT_CONTROLLER_ONLY=y`).

## Tunable constants

Both live in `main/Kconfig.projbuild` and are one line each:

- `DASH_CURRENT_DIVISOR` (default 100) — ISA shunt raw current is divided by
  this to give amps.
- `DASH_MPH_PER_RPM` (default 0.008) — speed is derived from motor RPM alone.

`DASH_SIMULATE_CAN` feeds synthetic frames through the normal decode path so the
dash can be exercised on the bench. Never enable it on the tractor.

## Logo

The dash reserves a 200x100 slot at the top left. Replace `main/assets/logo.png`
with the real logo at that size and rebuild; nothing else needs to change.
