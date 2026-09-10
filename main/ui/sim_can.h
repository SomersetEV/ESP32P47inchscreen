#pragma once

/*
 * Synthetic CAN for the bench. Compiled to a no-op unless
 * CONFIG_DASH_SIMULATE_CAN is set, so calling this unconditionally is safe.
 */
void sim_can_start(void);
