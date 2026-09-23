#pragma once
#include "vehicle_state.h"

/*
 * can_decode.h
 *
 * Turns raw frames into vehicle_state, using the same formulas the Arduino
 * Due + Nextion dash used so the numbers on the new screen match the old one.
 *
 * Called from the logger task only — it is the single writer of vehicle_state.
 */

// Parse the Kconfig-derived constants. Call once before the first decode.
void can_decode_init(void);

// Decode one frame into vehicle_state. Unknown IDs are ignored (they are still
// logged raw). Always stamps last_frame_us, so the dash's staleness check sees
// bus activity even on IDs the dash does not display.
//
// The caller holds vehicle_state_lock() around this, so it must not block,
// allocate or log.
void can_decode_frame(const raw_can_log_t *f);
