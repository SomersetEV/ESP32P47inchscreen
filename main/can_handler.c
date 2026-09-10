/*
 * can_handler.c — CAN bus capture (listen-only)
 *
 * The RX path is deliberately as short as possible: the ISR pulls the frame
 * out of the peripheral and hands it straight to the logger's queue. There is
 * no intermediate task and no decoding — every frame on the bus is recorded
 * verbatim.
 *
 * Listen-only is non-negotiable for a logger: the node must never ACK or send
 * error frames, so it cannot disturb the vehicle bus it is observing.
 *
 * Timestamps are taken here in the ISR with esp_timer_get_time(), not from the
 * driver's timestamp feature. The plain ESP32 has no TWAI capture timer, so
 * that feature would fall back to exactly this call inside the driver — doing
 * it explicitly makes the source of the number visible rather than depending
 * on a driver fallback that silently changes on hardware with a real timer.
 *
 * Note what the number does and does not mean. It is taken when the ISR pulls
 * the frame out of the peripheral, so SD write latency never skews it. But the
 * ISR drains the whole RX FIFO in one pass, so a burst that had been sitting in
 * the FIFO is stamped at drain time, not at arrival time: several frames can
 * carry near-identical timestamps. Deltas are reliable at moderate bus load and
 * compress under bursts.
 */

#include "can_handler.h"
#include "can_logger.h"
#include "board.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "CAN";

#define CAN_BITRATE 500000

static twai_node_handle_t s_node          = NULL;
static volatile uint32_t  s_frame_count   = 0;
static volatile uint32_t  s_error_count   = 0;
static volatile uint32_t  s_bus_off_count = 0;

// Live bus-off state, as opposed to the cumulative count above. Set and cleared
// from the ISR on error-state transitions; read by the logger task to decide
// whether recovery still needs retrying.
static volatile bool      s_is_bus_off    = false;

// ── ISR: frame received ───────────────────────────────────────────────────────

static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                 const twai_rx_done_event_data_t *edata,
                                 void *user_ctx)
{
    (void)edata; (void)user_ctx;

    uint8_t      buf[8];
    twai_frame_t rx = { .buffer = buf, .buffer_len = sizeof(buf) };

    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) return false;

    s_frame_count++;

    // dlc is the raw DLC code; for classic CAN it equals the byte count, but
    // clamp anyway so a malformed frame can never overrun the buffer.
    uint8_t len = (uint8_t)(rx.header.dlc <= 8 ? rx.header.dlc : 8);

    can_frame_t f = {
        .us  = esp_timer_get_time(),
        .id  = rx.header.id,
        .dlc = len,
        .ext = (uint8_t)rx.header.ide,
        .rtr = (uint8_t)rx.header.rtr,
    };
    // A remote frame carries no payload; buf is untouched in that case.
    if (!f.rtr) memcpy(f.data, buf, len);

    BaseType_t woken = pdFALSE;
    can_logger_submit_from_isr(&f, &woken);
    return woken == pdTRUE;
}

// ── ISR: bus error ────────────────────────────────────────────────────────────

static bool IRAM_ATTR on_error(twai_node_handle_t handle,
                               const twai_error_event_data_t *edata,
                               void *user_ctx)
{
    (void)handle; (void)edata; (void)user_ctx;
    s_error_count++;
    return false;
}

// ── ISR: error-state change ───────────────────────────────────────────────────
//
// Bus-off means the node has taken itself offline and will not receive another
// frame until recovered. An unattended logger must not stay deaf after a
// transient fault (a disconnected stub, a shorted pair), so flag it for the
// logger task to recover — twai_node_recover() is not ISR-safe.
//
// Recovery is asynchronous: twai_node_recover() only issues the command, and
// the hardware needs 128 consecutive occurrences of 11 recessive bits before it
// returns to error-active. That completion arrives as a second call here, which
// is what clears s_is_bus_off. Until then the logger keeps retrying.

static bool IRAM_ATTR on_state_change(twai_node_handle_t handle,
                                      const twai_state_change_event_data_t *edata,
                                      void *user_ctx)
{
    (void)handle; (void)user_ctx;
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        s_bus_off_count++;
        s_is_bus_off = true;
        can_logger_notify_bus_off_from_isr();
    } else if (edata->old_sta == TWAI_ERROR_BUS_OFF) {
        // Back on the bus — recovery finished.
        s_is_bus_off = false;
    }
    return false;
}

// ── Recovery, called from the logger task ─────────────────────────────────────

/*
 * The driver rejects this with ESP_ERR_INVALID_STATE unless the node is
 * currently bus-off, so a failure here is expected and must leave the caller
 * free to retry rather than losing the request. Returning the outcome is what
 * lets the logger task keep the retry armed.
 */
bool can_handler_recover(void)
{
    if (!s_node) return false;

    esp_err_t err = twai_node_recover(s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus-off recovery request failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGW(TAG, "Bus-off - recovery started, waiting for bus to go idle");
    return true;
}

bool can_handler_is_bus_off(void) { return s_is_bus_off; }

// ── Start ─────────────────────────────────────────────────────────────────────

static bool start_node(void)
{
    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx                = CAN_TX_PIN,
            .rx                = CAN_RX_PIN,
            .quanta_clk_out    = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = { .bitrate = CAN_BITRATE },
        // Timestamps are taken in on_rx_done, so the driver's timestamp
        // feature is left off. See the file header.
        .flags = { .enable_listen_only = 1 },
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(err));
        return false;
    }

    twai_event_callbacks_t cbs = {
        .on_rx_done      = on_rx_done,
        .on_error        = on_error,
        .on_state_change = on_state_change,
    };
    err = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Callback registration failed: %s", esp_err_to_name(err));
        return false;
    }

    err = twai_node_enable(s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "TWAI started - %d kbps, listen-only", CAN_BITRATE / 1000);
    return true;
}

/*
 * The TWAI interrupt is allocated on whichever core calls
 * twai_new_node_onchip(), so the install runs in a short-lived task pinned to
 * core 1. That keeps the RX ISR off core 0, where the logger task does its
 * blocking SD writes.
 */
static volatile bool s_start_done = false;
static volatile bool s_start_ok   = false;

static void start_task(void *arg)
{
    (void)arg;
    s_start_ok   = start_node();
    s_start_done = true;
    vTaskDelete(NULL);
}

bool can_handler_start(void)
{
    s_start_done = false;
    s_start_ok   = false;

    if (xTaskCreatePinnedToCore(start_task, "can_init", 4096, NULL,
                                6, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Could not create CAN init task");
        return false;
    }

    while (!s_start_done) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return s_start_ok;
}

// ── Counters ──────────────────────────────────────────────────────────────────

uint32_t can_handler_frame_count(void)   { return s_frame_count; }
uint32_t can_handler_error_count(void)   { return s_error_count; }
uint32_t can_handler_bus_off_count(void) { return s_bus_off_count; }
