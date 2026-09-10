#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * can_handler.h — CAN bus capture
 *
 * Brings up the TWAI peripheral in listen-only mode and pushes every received
 * frame straight into the logger queue from the ISR. No decoding: this
 * firmware is a pure bus recorder.
 */

// Bring up the TWAI peripheral. Returns false if the driver refuses to start.
bool can_handler_start(void);

/*
 * Ask the node to leave bus-off. Must be called from a task, not an ISR.
 *
 * Returns true if recovery was *started*. It does not complete synchronously:
 * the hardware needs 128 consecutive occurrences of 11 recessive bits before
 * it is active again. Poll can_handler_is_bus_off() to see when that lands.
 */
bool can_handler_recover(void);

// True while the node is bus-off and therefore receiving nothing.
bool can_handler_is_bus_off(void);

// Total frames accepted from the bus since boot.
uint32_t can_handler_frame_count(void);

// Bus error/state counters, for the periodic health line.
uint32_t can_handler_error_count(void);
uint32_t can_handler_bus_off_count(void);
