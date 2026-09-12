#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mcp2517fd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_TRACKER_MAX_IDS 256U

typedef struct {
    uint32_t id;
    bool     is_extended;
    bool     is_fd;
    bool     is_brs;
    uint8_t  last_len;
    uint8_t  last_data[64];
    uint64_t count;
    uint64_t prev_count;
    uint32_t rate_per_sec;
    uint32_t last_time_ms;
    bool     occupied;
} can_id_stat_t;

typedef struct {
    uint64_t total_messages;
    uint32_t total_rate_per_sec;
    uint32_t unique_ids_count;
    uint32_t overflow_count;
} can_tracker_stats_t;

/**
 * @brief Initialize the CAN message tracker
 */
void can_tracker_init(void);

/**
 * @brief Record a newly received CAN FD frame into tracker stats
 * @param frame Pointer to received CAN FD frame
 */
void can_tracker_record_frame(const canfd_frame_t *frame);

/**
 * @brief Print the formatted traffic table and full diagnostics to console
 * @param diag Pointer to current MCP2517FD diagnostics
 */
void can_tracker_print_summary(const mcp2517fd_diag_t *diag);

/**
 * @brief Get overall statistics snapshot
 * @param stats Pointer to store statistics
 */
void can_tracker_get_stats(can_tracker_stats_t *stats);

#ifdef __cplusplus
}
#endif
