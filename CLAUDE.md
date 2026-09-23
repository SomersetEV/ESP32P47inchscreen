# Design constraints

Carried from the pure CAN logger, which these rules kept reliable in a vehicle.

## The logger must survive power loss at any instant

There is no operator and no shutdown sequence. Power is cut by a key switch
mid-write and that is the normal case, not an error path.

- Both files are `fflush` + `fsync`'d every 1000 ms. A power cut costs at most
  one second of data.
- Never buffer log data anywhere that is not flushed on that cycle.
- The session id is claimed and committed to NVS **before** any file is opened,
  so a crash cannot make two boots share a file name.
- On a full card, delete the oldest `canlog_`/`snap_` pair — never the active
  session.

## Core split

- Core 1: TWAI RX ISR only. The TWAI interrupt lands on whichever core calls
  `twai_new_node_onchip()`, so that call is made from a short-lived core-1 task.
- Core 0: all file I/O, LVGL and BLE.

Do not move file I/O onto core 1; it blocks and the RX queue overruns.

## vehicle_state has one writer

The logger task decodes each dequeued frame and is the only writer. The UI and
BLE read it inside a short critical section. Do not write to it from the UI, a
timer, or the BLE task.

## LVGL is not thread-safe

Every `lv_*` call sits between `bsp_display_lock()` and `bsp_display_unlock()`.

## Phone app compatibility is fixed

The Flutter app's parser is not being changed. The BLE command set, the
`snap_NNNN.csv` column layout, and the TRIP_START / TRIP_END row format are
byte-compatible with the LilyGo telematics firmware. Add columns only at the
end.

## Build hygiene

- Never run `idf.py menuconfig`. Configuration goes in `sdkconfig.defaults`;
  delete `sdkconfig` after changing it.
- Never run `idf.py monitor` — it does not return. Use `tools/capture_log.ps1`.
- Never patch anything under `managed_components/`. Work around component bugs
  from the top-level `CMakeLists.txt` instead.
  - Exception: `waveshare__esp32_p4_wifi6_touch_lcd_7b/esp32_p4_wifi6_touch_lcd_7b.c`
    hardcodes the LVGL draw buffer to `buffer_height = 50`, `use_psram = false`,
    `enable_ppa_accel = false` inside `bsp_display_lcd_init()`, with no field on
    `bsp_display_cfg_t` and no Kconfig symbol to reach it from the top level.
    That 1024x50 SRAM buffer forced every full-screen LVGL animation (e.g. the
    splash fade) into ~12 partial-buffer passes per frame and made them stutter.
    Patched in place to `buffer_height = BSP_LCD_V_RES`, `use_psram = true`,
    `enable_ppa_accel = true`, approved explicitly by the user as a deliberate,
    one-off exception to this rule. If this component is ever updated from
    upstream, reapply the same three field changes.
- CAN is listen-only. This firmware must never transmit on the tractor bus.
