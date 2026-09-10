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

### One SDMMC controller, two slots

This is the subtlest thing in the project. The P4 has **one** SDMMC controller
driving two slots, and this board uses both: the microSD card on slot 0, and the
ESP32-C6 (Wi-Fi/BLE, over esp_hosted) on slot 1.

IDF 6.0's legacy `sdmmc_host_init()` creates a controller into a file-static
handle with no guard against a second call, and the claim function hands out the
single controller exactly once. So the second caller gets `ESP_ERR_NOT_FOUND`
("no available sd host controller") and its slot never comes up. esp_hosted
calls it first, from a `__attribute__((constructor))` that runs before
`app_main`, so the SD card always lost — with an error message that points
nowhere near the real cause.

The rest of the driver is written for a shared controller (the same static
handle serves both slot handles, and per-slot init is separate), so the only
thing missing is the guard. [main/sd_host_init_shim.c](main/sd_host_init_shim.c)
adds it by wrapping that one function via `-Wl,--wrap`, so nothing under
`managed_components/` is patched. Remove it and BLE and the SD card cannot both
work.

`sd_store_mount()` also does the SDMMC mount itself rather than calling
`bsp_sdcard_mount()`, which acquires on-chip LDO channel 4 on every call and
never releases it on failure — that makes the second and every later retry fail
with a misleading "can't acquire the channel" instead of the real fault.

### ESP32-C6 firmware

BLE runs as a NimBLE host on the P4 with the controller on the ESP32-C6, linked
by esp_hosted over SDIO. Host and slave must be on the same major line. This
project uses host **2.0.17**, and **the factory C6 firmware on this board works
with it** — confirmed by the phone app connecting and syncing. If a future board
fails the handshake and never advertises, reflash the C6 through its UART header
with the matching esp_hosted `slave` example (target esp32c6, SDIO,
`CONFIG_BT_ENABLED=y`, `CONFIG_BT_CONTROLLER_ONLY=y`).

### No RTC fitted

The plan expected a PCF85063A at 0x51. A bus scan at boot reports only 0x18
(ES8311 codec), 0x40 and 0x5D (GT911 touch), so **this board has no RTC**. The
firmware handles that: the clock stays invalid, the dash shows `--:--`, and the
`unix_ms` / `unix_time` columns are 0 until the phone sends `TIME`. After that
the time is correct for the rest of the power cycle but does not survive a
power-off. Fitting a PCF85063A at 0x51 with its coin cell is all the driver
needs — `rtc_time.c` is already written and wired to `TIME`.

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

## Log files

Each power cycle claims a session number from NVS (namespace `telematics`, key
`session_id`) **before** any file is opened, so a power cut can never hand the
next boot a number already in use. Both files carry that number.

`canlog_NNNN.csv` — every frame on the bus:

```
us,unix_ms,id,ext,rtr,dlc,data
```

`unix_ms` is 0 until the clock is set. A gap in the capture is recorded in the
file itself as a `DROPPED=n` row rather than only on the console.

`snap_NNNN.csv` — 1 Hz decoded snapshot. The first 14 columns are the telematics
`SNAP1` layout byte for byte, because the phone app's parser is not being
changed; `charger_temp_c10` and `unix_time` are appended after them. Trip
markers and the end-of-trip summary are written here too.

Both files are `fflush`ed and `fsync`ed every second, so a power cut costs at
most one second from either. Power is expected to be cut without warning — there
is no shutdown sequence and nothing depends on one.

When the card falls below 10% free, the lowest-numbered session is deleted,
both files together. The active session is never deleted.

## Gate status

| Gate | What it covers | Status |
|---|---|---|
| 0 | Builds, dependencies resolve, blank app boots | Passed on hardware |
| 1 | Splash, fade, dash animates with simulated CAN | Passed on hardware (user-confirmed on screen) |
| 2 | RTC probe reports found or not-found | Passed — reports not fitted, see above |
| 3a | SD mounts, session files open, TWAI starts | **Not passed — no SD card fitted.** Mount reports "no card responding" and retries every 5 s. Everything up to the card responding is verified. |
| 3b | Live CAN frames decode and log | **Not tested — needs a bus and a card** |
| 4a | BLE advertises | Passed on hardware |
| 4b | Phone app connects and syncs | Passed in part: the app connected and ran TIME, LIST, STATUS and WIFI_MODE. A full session sync (GET/DONE) and a trip start/end still need a card. |

To finish 3a/3b: fit a FAT32 microSD card, reflash, and check for `SD mounted`,
`Logging to canlog_NNNN.csv`, `Snapshots to snap_NNNN.csv` and
`TWAI started - 500 kbps, listen-only`, then connect the PH2.0 CAN header to the
tractor and confirm the 30 s health line shows frames with 0 dropped.
