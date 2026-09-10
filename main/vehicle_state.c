#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#include "vehicle_state.h"

/*
 * One writer (the logger task), several readers. The struct is larger than a
 * word, so readers that need a coherent set of values take a copy inside a
 * critical section rather than reading fields one at a time.
 */

static vehicle_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

void vehicle_state_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
}

vehicle_state_t *vehicle_state_get(void)
{
    return &s_state;
}

void vehicle_state_snapshot(vehicle_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    memcpy(out, &s_state, sizeof(*out));
    portEXIT_CRITICAL(&s_lock);
}

bool vehicle_state_is_fresh(uint32_t stale_ms)
{
    portENTER_CRITICAL(&s_lock);
    uint64_t last = s_state.last_frame_us;
    portEXIT_CRITICAL(&s_lock);

    if (last == 0) {
        return false;   // nothing has ever arrived
    }
    return (esp_timer_get_time() - last) < ((uint64_t)stale_ms * 1000);
}
