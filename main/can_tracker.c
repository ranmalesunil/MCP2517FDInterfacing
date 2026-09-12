#include "can_tracker.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static can_id_stat_t s_id_table[CAN_TRACKER_MAX_IDS];
static uint32_t s_unique_ids = 0U;
static uint64_t s_total_messages = 0ULL;
static uint64_t s_prev_total_messages = 0ULL;
static uint32_t s_total_rate = 0U;
static SemaphoreHandle_t s_tracker_mutex = NULL;
static int64_t s_last_report_us = 0LL;

static inline uint32_t make_key(uint32_t id, bool is_ext)
{
    return (is_ext ? (id | 0x80000000U) : (id & 0x7FFU));
}

static inline uint32_t hash_key(uint32_t key)
{
    uint32_t k = key;
    k = ((k >> 16U) ^ k) * 0x45d9f3bU;
    k = ((k >> 16U) ^ k) * 0x45d9f3bU;
    k = (k >> 16U) ^ k;
    return (k % CAN_TRACKER_MAX_IDS);
}

void can_tracker_init(void)
{
    if (s_tracker_mutex == NULL) {
        s_tracker_mutex = xSemaphoreCreateMutex();
    }
    (void)memset(s_id_table, 0, sizeof(s_id_table));
    s_unique_ids = 0U;
    s_total_messages = 0ULL;
    s_prev_total_messages = 0ULL;
    s_total_rate = 0U;
    s_last_report_us = esp_timer_get_time();
}

void can_tracker_record_frame(const canfd_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    uint32_t key = make_key(frame->id, frame->is_extended);
    uint32_t idx = hash_key(key);
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

    if (xSemaphoreTake(s_tracker_mutex, pdMS_TO_TICKS(50U)) == pdTRUE) {
        s_total_messages++;
        for (uint32_t i = 0U; i < CAN_TRACKER_MAX_IDS; i++) {
            uint32_t probe_idx = (idx + i) % CAN_TRACKER_MAX_IDS;
            if (!s_id_table[probe_idx].occupied) {
                /* New unique ID */
                s_id_table[probe_idx].id = frame->id;
                s_id_table[probe_idx].is_extended = frame->is_extended;
                s_id_table[probe_idx].is_fd = frame->is_fd;
                s_id_table[probe_idx].is_brs = frame->is_brs;
                s_id_table[probe_idx].last_len = frame->len;
                uint8_t copy_bytes = (frame->len > 64U) ? 64U : frame->len;
                (void)memcpy(s_id_table[probe_idx].last_data, frame->data, copy_bytes);
                s_id_table[probe_idx].count = 1ULL;
                s_id_table[probe_idx].prev_count = 0ULL;
                s_id_table[probe_idx].rate_per_sec = 0U;
                s_id_table[probe_idx].last_time_ms = now_ms;
                s_id_table[probe_idx].occupied = true;
                s_unique_ids++;
                break;
            } else if ((s_id_table[probe_idx].id == frame->id) &&
                       (s_id_table[probe_idx].is_extended == frame->is_extended)) {
                /* Existing ID */
                s_id_table[probe_idx].count++;
                s_id_table[probe_idx].is_fd = frame->is_fd;
                s_id_table[probe_idx].is_brs = frame->is_brs;
                s_id_table[probe_idx].last_len = frame->len;
                uint8_t copy_bytes = (frame->len > 64U) ? 64U : frame->len;
                (void)memcpy(s_id_table[probe_idx].last_data, frame->data, copy_bytes);
                s_id_table[probe_idx].last_time_ms = now_ms;
                break;
            } else {
                /* Continue probing */
            }
        }
        (void)xSemaphoreGive(s_tracker_mutex);
    } else {
        s_total_messages++;
    }
}

void can_tracker_print_summary(const mcp2517fd_diag_t *diag)
{
    /* Local copy of table so we do NOT hold the mutex during slow UART printf() calls */
    static can_id_stat_t s_id_table_copy[CAN_TRACKER_MAX_IDS];
    uint32_t unique_ids_copy = 0U;
    uint64_t total_msgs_copy = 0ULL;
    uint32_t total_rate_copy = 0U;

    if (xSemaphoreTake(s_tracker_mutex, pdMS_TO_TICKS(50U)) != pdTRUE) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    int64_t delta_us = now_us - s_last_report_us;
    if (delta_us <= 50000LL) {
        delta_us = 1000000LL; /* Guard against divide by zero or too small intervals */
    }
    s_last_report_us = now_us;

    uint64_t delta_total = s_total_messages - s_prev_total_messages;
    s_total_rate = (uint32_t)((delta_total * 1000000ULL) / (uint64_t)delta_us);
    s_prev_total_messages = s_total_messages;

    for (uint32_t i = 0U; i < CAN_TRACKER_MAX_IDS; i++) {
        if (s_id_table[i].occupied) {
            uint64_t delta = s_id_table[i].count - s_id_table[i].prev_count;
            s_id_table[i].rate_per_sec = (uint32_t)((delta * 1000000ULL) / (uint64_t)delta_us);
            s_id_table[i].prev_count = s_id_table[i].count;
        }
    }

    /* Fast memory snapshot (takes < 1 microsecond) */
    (void)memcpy(s_id_table_copy, s_id_table, sizeof(s_id_table_copy));
    unique_ids_copy = s_unique_ids;
    total_msgs_copy = s_total_messages;
    total_rate_copy = s_total_rate;

    /* RELEASE MUTEX IMMEDIATELY: RX task will NEVER be blocked by UART! */
    (void)xSemaphoreGive(s_tracker_mutex);

    uint32_t overflows = mcp2517fd_get_rx_overflow_count();

    const char *state_str = "ACTIVE (OK)";
    if (diag != NULL) {
        if (diag->txbo) {
            state_str = "BUS-OFF";
        } else if (diag->txbp || diag->rxbp) {
            state_str = "ERROR-PASSIVE";
        } else if (diag->txwarn || diag->rxwarn) {
            state_str = "ERROR-WARN";
        } else {
            /* State remains ACTIVE */
        }
    }

    uint8_t tec = (diag != NULL) ? diag->tec : 0U;
    uint8_t rec = (diag != NULL) ? diag->rec : 0U;

    (void)printf("\n");
    (void)printf("+===================================================================================================+\n");
    (void)printf("|                                  MCP2517FD CAN FD TRAFFIC MONITOR                                 |\n");
    (void)printf("| Total Msgs: %-9" PRIu64 " | Rate: %-5" PRIu32 " msg/s | Unique IDs: %-3" PRIu32 " | Overflows: %-3" PRIu32 " | Bus: %-13s |\n",
                 total_msgs_copy, total_rate_copy, unique_ids_copy, overflows, state_str);
    (void)printf("| TEC (TxErr): %-3u | REC (RxErr): %-3u | TREC: 0x%06" PRIX32 " | BDIAG1: 0x%06" PRIX32 "                      |\n",
                 tec, rec, (diag != NULL) ? (diag->trec_raw & 0xFFFFFFU) : 0U, (diag != NULL) ? (diag->bdiag1_raw & 0xFFFFFFU) : 0U);

    /* Print live diagnostic hint if errors detected */
    if (diag != NULL) {
        if (diag->nbit0_err || diag->dbit0_err) {
            (void)printf("| [!] CRITICAL: NBIT0/DBIT0 ERROR (Transceiver loopback failed: sent dominant '0', monitored '1')  |\n");
            (void)printf("|     -> Hardware Cause: Transceiver has NO 5V VCC power, OR Transceiver STBY pin is NOT tied to GND!|\n");
            (void)printf("|     -> Required Fixes:                                                                            |\n");
            (void)printf("|        1. Connect ESP32 5V (VBUS/VIN) to MCP2517FD module 5V/VCC pin                             |\n");
            (void)printf("|        2. Connect module STBY / STB / S pin directly to GND (0 V)                                |\n");
            (void)printf("|        3. Verify 120-ohm termination jumper is installed across CAN_H and CAN_L                   |\n");
        } else if (diag->nack_err) {
            (void)printf("| [!] BUS WARNING: NACK_ERR (ESP32 transmitted, but NO node acknowledged on CAN bus)                |\n");
            (void)printf("|     -> Check: Is PCAN connected? CAN_H/CAN_L wired correctly? PCAN bitrate set to 500k/2M?       |\n");
            (void)printf("|     -> Check: 120-ohm termination resistor present at both ends of CAN bus?                      |\n");
        } else {
            /* No bit0 or nack error */
        }

        if (diag->nstuff_err || diag->ncrc_err) {
            (void)printf("| [!] BIT TIMING ERROR: STUFF/CRC ERROR (Baud rate / clock mismatch!)                               |\n");
            (void)printf("|     -> Check: Is the board crystal 20MHz instead of 40MHz? Run menuconfig to select 20MHz.        |\n");
        }
        if (diag->txbo) {
            (void)printf("| [!] BUS-OFF: Transceiver entered Bus-Off due to persistent bus errors.                            |\n");
        }
    }

    (void)printf("+------------+-------+----+-----+---------+-----------------+----------+---------------------------+\n");
    (void)printf("|   CAN ID   | Type  | FD | BRS | Length  |   Total Count   | Rate/sec | Last Data (First 8 bytes) |\n");
    (void)printf("+------------+-------+----+-----+---------+-----------------+----------+---------------------------+\n");

    if (unique_ids_copy == 0U) {
        (void)printf("|                        Waiting for CAN / CAN FD messages on the bus...                            |\n");
    } else {
        for (uint32_t i = 0U; i < CAN_TRACKER_MAX_IDS; i++) {
            if (s_id_table_copy[i].occupied) {
                char hex_preview[32] = {0};
                uint8_t preview_bytes = (s_id_table_copy[i].last_len > 8U) ? 8U : s_id_table_copy[i].last_len;
                for (uint8_t b = 0U; b < preview_bytes; b++) {
                    char b_str[4] = {0};
                    (void)snprintf(b_str, sizeof(b_str), "%02X ", s_id_table_copy[i].last_data[b]);
                    (void)strncat(hex_preview, b_str, sizeof(hex_preview) - strlen(hex_preview) - 1U);
                }
                if (s_id_table_copy[i].last_len > 8U) {
                    (void)strncat(hex_preview, "..", sizeof(hex_preview) - strlen(hex_preview) - 1U);
                }

                (void)printf("| 0x%08" PRIX32 " | %-5s | %-2s | %-3s | %2u B    | %-15" PRIu64 " | %-8" PRIu32 " | %-25s |\n",
                             s_id_table_copy[i].id,
                             s_id_table_copy[i].is_extended ? "EXT" : "STD",
                             s_id_table_copy[i].is_fd ? "FD" : "2.0",
                             s_id_table_copy[i].is_brs ? "YES" : "NO",
                             s_id_table_copy[i].last_len,
                             s_id_table_copy[i].count,
                             s_id_table_copy[i].rate_per_sec,
                             hex_preview);
            }
        }
    }
    (void)printf("+===================================================================================================+\n");
}

void can_tracker_get_stats(can_tracker_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    if (xSemaphoreTake(s_tracker_mutex, pdMS_TO_TICKS(20U)) == pdTRUE) {
        stats->total_messages = s_total_messages;
        stats->total_rate_per_sec = s_total_rate;
        stats->unique_ids_count = s_unique_ids;
        stats->overflow_count = mcp2517fd_get_rx_overflow_count();
        (void)xSemaphoreGive(s_tracker_mutex);
    }
}
