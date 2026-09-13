#include "usb_protocol.h"
#include <string.h>
#include "esp_timer.h"

typedef enum {
    PARSE_STATE_SYNC1 = 0,
    PARSE_STATE_SYNC2,
    PARSE_STATE_CMD,
    PARSE_STATE_SEQ,
    PARSE_STATE_LEN_MSB,
    PARSE_STATE_LEN_LSB,
    PARSE_STATE_PAYLOAD,
    PARSE_STATE_CRC_MSB,
    PARSE_STATE_CRC_LSB
} proto_parse_state_t;

static proto_parse_state_t s_parse_state = PARSE_STATE_SYNC1;
static proto_packet_t s_rx_packet;
static uint16_t s_payload_idx = 0U;

uint16_t proto_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if (data != NULL) {
        for (size_t i = 0U; i < length; i++) {
            crc ^= ((uint16_t)data[i] << 8U);
            for (uint8_t bit = 0U; bit < 8U; bit++) {
                if ((crc & 0x8000U) != 0U) {
                    crc = (uint16_t)((crc << 1U) ^ 0x1021U);
                } else {
                    crc = (uint16_t)(crc << 1U);
                }
            }
        }
    }
    return crc;
}

void proto_parse_reset(void)
{
    s_parse_state = PARSE_STATE_SYNC1;
    s_payload_idx = 0U;
    (void)memset(&s_rx_packet, 0, sizeof(s_rx_packet));
}

bool proto_parse_byte(uint8_t byte, proto_packet_t *out_packet)
{
    bool packet_ready = false;

    switch (s_parse_state) {
    case PARSE_STATE_SYNC1:
        if (byte == PROTO_SYNC_BYTE_1) {
            s_parse_state = PARSE_STATE_SYNC2;
        }
        break;

    case PARSE_STATE_SYNC2:
        if (byte == PROTO_SYNC_BYTE_2) {
            s_parse_state = PARSE_STATE_CMD;
        } else if (byte != PROTO_SYNC_BYTE_1) {
            s_parse_state = PARSE_STATE_SYNC1;
        }
        break;

    case PARSE_STATE_CMD:
        s_rx_packet.cmd = byte;
        s_parse_state = PARSE_STATE_SEQ;
        break;

    case PARSE_STATE_SEQ:
        s_rx_packet.seq = byte;
        s_parse_state = PARSE_STATE_LEN_MSB;
        break;

    case PARSE_STATE_LEN_MSB:
        s_rx_packet.len = (uint16_t)((uint16_t)byte << 8U);
        s_parse_state = PARSE_STATE_LEN_LSB;
        break;

    case PARSE_STATE_LEN_LSB:
        s_rx_packet.len |= (uint16_t)byte;
        if (s_rx_packet.len > PROTO_MAX_PAYLOAD_SIZE) {
            /* Payload size exceeds buffer limit, reset parser */
            s_parse_state = PARSE_STATE_SYNC1;
        } else if (s_rx_packet.len == 0U) {
            s_parse_state = PARSE_STATE_CRC_MSB;
        } else {
            s_payload_idx = 0U;
            s_parse_state = PARSE_STATE_PAYLOAD;
        }
        break;

    case PARSE_STATE_PAYLOAD:
        s_rx_packet.payload[s_payload_idx] = byte;
        s_payload_idx++;
        if (s_payload_idx >= s_rx_packet.len) {
            s_parse_state = PARSE_STATE_CRC_MSB;
        }
        break;

    case PARSE_STATE_CRC_MSB:
        s_rx_packet.crc = (uint16_t)((uint16_t)byte << 8U);
        s_parse_state = PARSE_STATE_CRC_LSB;
        break;

    case PARSE_STATE_CRC_LSB:
        s_rx_packet.crc |= (uint16_t)byte;

        /* Verify CRC: Compute CRC over [CMD, SEQ, LEN_MSB, LEN_LSB, PAYLOAD...] */
        {
            uint8_t hdr[4];
            hdr[0] = s_rx_packet.cmd;
            hdr[1] = s_rx_packet.seq;
            hdr[2] = (uint8_t)(s_rx_packet.len >> 8U);
            hdr[3] = (uint8_t)(s_rx_packet.len & 0xFFU);

            uint16_t calc_crc = 0xFFFFU;
            /* Feed header */
            for (size_t i = 0U; i < 4U; i++) {
                calc_crc ^= ((uint16_t)hdr[i] << 8U);
                for (uint8_t bit = 0U; bit < 8U; bit++) {
                    if ((calc_crc & 0x8000U) != 0U) {
                        calc_crc = (uint16_t)((calc_crc << 1U) ^ 0x1021U);
                    } else {
                        calc_crc = (uint16_t)(calc_crc << 1U);
                    }
                }
            }
            /* Feed payload */
            for (size_t i = 0U; i < (size_t)s_rx_packet.len; i++) {
                calc_crc ^= ((uint16_t)s_rx_packet.payload[i] << 8U);
                for (uint8_t bit = 0U; bit < 8U; bit++) {
                    if ((calc_crc & 0x8000U) != 0U) {
                        calc_crc = (uint16_t)((calc_crc << 1U) ^ 0x1021U);
                    } else {
                        calc_crc = (uint16_t)(calc_crc << 1U);
                    }
                }
            }

            if ((calc_crc == s_rx_packet.crc) && (out_packet != NULL)) {
                (void)memcpy(out_packet, &s_rx_packet, sizeof(proto_packet_t));
                packet_ready = true;
            }
        }
        s_parse_state = PARSE_STATE_SYNC1;
        break;

    default:
        s_parse_state = PARSE_STATE_SYNC1;
        break;
    }

    return packet_ready;
}

static esp_err_t proto_pack(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len,
                            uint8_t *out_buf, size_t *out_len)
{
    if ((out_buf == NULL) || (out_len == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_len = 8U + (size_t)len; /* 2 sync + 1 cmd + 1 seq + 2 len + len + 2 crc */
    out_buf[0] = PROTO_SYNC_BYTE_1;
    out_buf[1] = PROTO_SYNC_BYTE_2;
    out_buf[2] = cmd;
    out_buf[3] = seq;
    out_buf[4] = (uint8_t)(len >> 8U);
    out_buf[5] = (uint8_t)(len & 0xFFU);

    if ((payload != NULL) && (len > 0U)) {
        (void)memcpy(&out_buf[6], payload, len);
    }

    /* Compute CRC over header + payload: from offset 2 (cmd) to 6 + len */
    uint16_t crc = proto_crc16(&out_buf[2], 4U + (size_t)len);
    out_buf[6U + (size_t)len] = (uint8_t)(crc >> 8U);
    out_buf[7U + (size_t)len] = (uint8_t)(crc & 0xFFU);

    *out_len = total_len;
    return ESP_OK;
}

esp_err_t proto_serialize_ack(uint8_t cmd, uint8_t seq, uint8_t status, uint8_t *out_buf, size_t *out_len)
{
    uint8_t payload[2];
    payload[0] = cmd;
    payload[1] = status;
    uint8_t resp_cmd = (status == PROTO_STATUS_OK) ? PROTO_RSP_ACK : PROTO_RSP_NACK;
    return proto_pack(resp_cmd, seq, payload, sizeof(payload), out_buf, out_len);
}

esp_err_t proto_serialize_ping_rsp(uint8_t seq, uint8_t current_mode, uint32_t nominal_bps, uint32_t data_bps,
                                   uint8_t *out_buf, size_t *out_len)
{
    uint8_t payload[18];
    payload[0] = 1U;      /* FW Major */
    payload[1] = 0U;      /* FW Minor */
    payload[2] = 0U;      /* FW Patch */
    payload[3] = 0x53U;   /* Hardware Type: 'S' for ESP32-S3 + MCP2517FD */
    payload[4] = current_mode;
    payload[5] = 0U;      /* Reserved */

    payload[6] = (uint8_t)(nominal_bps & 0xFFU);
    payload[7] = (uint8_t)((nominal_bps >> 8U) & 0xFFU);
    payload[8] = (uint8_t)((nominal_bps >> 16U) & 0xFFU);
    payload[9] = (uint8_t)((nominal_bps >> 24U) & 0xFFU);

    payload[10] = (uint8_t)(data_bps & 0xFFU);
    payload[11] = (uint8_t)((data_bps >> 8U) & 0xFFU);
    payload[12] = (uint8_t)((data_bps >> 16U) & 0xFFU);
    payload[13] = (uint8_t)((data_bps >> 24U) & 0xFFU);

    uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000LL);
    payload[14] = (uint8_t)(uptime_sec & 0xFFU);
    payload[15] = (uint8_t)((uptime_sec >> 8U) & 0xFFU);
    payload[16] = (uint8_t)((uptime_sec >> 16U) & 0xFFU);
    payload[17] = (uint8_t)((uptime_sec >> 24U) & 0xFFU);

    return proto_pack(PROTO_RSP_PING, seq, payload, sizeof(payload), out_buf, out_len);
}

esp_err_t proto_serialize_rx_frame(const canfd_frame_t *frame, uint64_t timestamp_us,
                                   uint8_t *out_buf, size_t *out_len)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[80];
    /* Microsecond timestamp: 8 bytes LE */
    payload[0] = (uint8_t)(timestamp_us & 0xFFU);
    payload[1] = (uint8_t)((timestamp_us >> 8U) & 0xFFU);
    payload[2] = (uint8_t)((timestamp_us >> 16U) & 0xFFU);
    payload[3] = (uint8_t)((timestamp_us >> 24U) & 0xFFU);
    payload[4] = (uint8_t)((timestamp_us >> 32U) & 0xFFU);
    payload[5] = (uint8_t)((timestamp_us >> 40U) & 0xFFU);
    payload[6] = (uint8_t)((timestamp_us >> 48U) & 0xFFU);
    payload[7] = (uint8_t)((timestamp_us >> 56U) & 0xFFU);

    /* CAN ID: 4 bytes LE */
    payload[8]  = (uint8_t)(frame->id & 0xFFU);
    payload[9]  = (uint8_t)((frame->id >> 8U) & 0xFFU);
    payload[10] = (uint8_t)((frame->id >> 16U) & 0xFFU);
    payload[11] = (uint8_t)((frame->id >> 24U) & 0xFFU);

    /* Flags: 1 byte */
    uint8_t flags = 0U;
    if (frame->is_extended) {
        flags |= PROTO_FRAME_FLAG_EXT;
    }
    if (frame->is_fd) {
        flags |= PROTO_FRAME_FLAG_FD;
    }
    if (frame->is_brs) {
        flags |= PROTO_FRAME_FLAG_BRS;
    }
    if (frame->is_rtr) {
        flags |= PROTO_FRAME_FLAG_RTR;
    }
    payload[12] = flags;

    /* Length: 1 byte */
    uint8_t len = (frame->len > 64U) ? 64U : frame->len;
    payload[13] = len;

    /* Filter Index & Reserved */
    payload[14] = frame->filter_idx;
    payload[15] = 0U;

    /* Data bytes */
    if (len > 0U) {
        (void)memcpy(&payload[16], frame->data, len);
    }

    uint16_t total_payload = 16U + (uint16_t)len;
    return proto_pack(PROTO_MSG_RX_FRAME, 0U, payload, total_payload, out_buf, out_len);
}

esp_err_t proto_serialize_tx_confirm(uint32_t frame_id, uint8_t seq, uint64_t timestamp_us, uint8_t status,
                                     uint8_t *out_buf, size_t *out_len)
{
    uint8_t payload[14];
    payload[0] = (uint8_t)(timestamp_us & 0xFFU);
    payload[1] = (uint8_t)((timestamp_us >> 8U) & 0xFFU);
    payload[2] = (uint8_t)((timestamp_us >> 16U) & 0xFFU);
    payload[3] = (uint8_t)((timestamp_us >> 24U) & 0xFFU);
    payload[4] = (uint8_t)((timestamp_us >> 32U) & 0xFFU);
    payload[5] = (uint8_t)((timestamp_us >> 40U) & 0xFFU);
    payload[6] = (uint8_t)((timestamp_us >> 48U) & 0xFFU);
    payload[7] = (uint8_t)((timestamp_us >> 56U) & 0xFFU);

    payload[8]  = (uint8_t)(frame_id & 0xFFU);
    payload[9]  = (uint8_t)((frame_id >> 8U) & 0xFFU);
    payload[10] = (uint8_t)((frame_id >> 16U) & 0xFFU);
    payload[11] = (uint8_t)((frame_id >> 24U) & 0xFFU);

    payload[12] = seq;
    payload[13] = status;

    return proto_pack(PROTO_MSG_TX_CONFIRM, seq, payload, sizeof(payload), out_buf, out_len);
}

esp_err_t proto_serialize_diag(const mcp2517fd_diag_t *diag, uint32_t rx_overflow_count,
                               uint32_t rx_count, uint32_t tx_count,
                               uint8_t *out_buf, size_t *out_len)
{
    if (diag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[22];
    payload[0] = diag->tec;
    payload[1] = diag->rec;

    uint16_t flags = 0U;
    if (diag->txbo)      { flags |= (1U << 0U); }
    if (diag->txbp)      { flags |= (1U << 1U); }
    if (diag->rxbp)      { flags |= (1U << 2U); }
    if (diag->txwarn)    { flags |= (1U << 3U); }
    if (diag->rxwarn)    { flags |= (1U << 4U); }
    if (diag->nbit0_err) { flags |= (1U << 5U); }
    if (diag->nbit1_err) { flags |= (1U << 6U); }
    if (diag->nack_err)  { flags |= (1U << 7U); }
    if (diag->nstuff_err){ flags |= (1U << 8U); }
    if (diag->ncrc_err)  { flags |= (1U << 9U); }
    if (diag->dbit0_err) { flags |= (1U << 10U); }
    if (diag->dbit1_err) { flags |= (1U << 11U); }

    payload[2] = (uint8_t)(flags & 0xFFU);
    payload[3] = (uint8_t)((flags >> 8U) & 0xFFU);

    payload[4] = (uint8_t)(rx_overflow_count & 0xFFU);
    payload[5] = (uint8_t)((rx_overflow_count >> 8U) & 0xFFU);
    payload[6] = (uint8_t)((rx_overflow_count >> 16U) & 0xFFU);
    payload[7] = (uint8_t)((rx_overflow_count >> 24U) & 0xFFU);

    payload[8]  = (uint8_t)(rx_count & 0xFFU);
    payload[9]  = (uint8_t)((rx_count >> 8U) & 0xFFU);
    payload[10] = (uint8_t)((rx_count >> 16U) & 0xFFU);
    payload[11] = (uint8_t)((rx_count >> 24U) & 0xFFU);

    payload[12] = (uint8_t)(tx_count & 0xFFU);
    payload[13] = (uint8_t)((tx_count >> 8U) & 0xFFU);
    payload[14] = (uint8_t)((tx_count >> 16U) & 0xFFU);
    payload[15] = (uint8_t)((tx_count >> 24U) & 0xFFU);

    uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000LL);
    payload[16] = (uint8_t)(uptime_sec & 0xFFU);
    payload[17] = (uint8_t)((uptime_sec >> 8U) & 0xFFU);
    payload[18] = (uint8_t)((uptime_sec >> 16U) & 0xFFU);
    payload[19] = (uint8_t)((uptime_sec >> 24U) & 0xFFU);

    payload[20] = 0U; /* Reserved */
    payload[21] = 0U;

    return proto_pack(PROTO_MSG_BUS_DIAG, 0U, payload, sizeof(payload), out_buf, out_len);
}
