#ifndef USB_PROTOCOL_H_
#define USB_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "mcp2517fd.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Packet Framing Delimiters */
#define PROTO_SYNC_BYTE_1           0xAAU
#define PROTO_SYNC_BYTE_2           0x55U

/* Host-to-Device Command IDs (PC -> ESP32) */
#define PROTO_CMD_PING              0x01U
#define PROTO_CMD_SET_BITRATE       0x02U
#define PROTO_CMD_SET_MODE          0x03U
#define PROTO_CMD_SET_FILTER        0x04U
#define PROTO_CMD_TX_FRAME          0x05U
#define PROTO_CMD_BUS_CONTROL       0x06U
#define PROTO_CMD_GET_DIAG          0x07U
#define PROTO_CMD_RESET_STATS       0x08U

/* Device-to-Host Response / Notification IDs (ESP32 -> PC) */
#define PROTO_RSP_ACK               0x80U
#define PROTO_RSP_NACK              0x81U
#define PROTO_MSG_RX_FRAME          0x82U
#define PROTO_MSG_TX_CONFIRM        0x83U
#define PROTO_MSG_BUS_DIAG          0x84U
#define PROTO_RSP_PING              0x85U

/* Protocol Status Codes */
#define PROTO_STATUS_OK             0x00U
#define PROTO_STATUS_ERR_GENERIC    0x01U
#define PROTO_STATUS_ERR_INVALID_CMD 0x02U
#define PROTO_STATUS_ERR_INVALID_ARG 0x03U
#define PROTO_STATUS_ERR_BUS_BUSY   0x04U
#define PROTO_STATUS_ERR_TIMEOUT    0x05U
#define PROTO_STATUS_ERR_CRC        0x06U

/* Protocol Maximum Payload Size */
#define PROTO_MAX_PAYLOAD_SIZE      128U

/* Frame Flags (Bitmask) */
#define PROTO_FRAME_FLAG_EXT        (1U << 0U)  /* 29-bit Extended ID */
#define PROTO_FRAME_FLAG_FD         (1U << 1U)  /* CAN FD Frame */
#define PROTO_FRAME_FLAG_BRS        (1U << 2U)  /* Bit Rate Switch enabled */
#define PROTO_FRAME_FLAG_RTR        (1U << 3U)  /* Remote Transmission Request */
#define PROTO_FRAME_FLAG_ESI        (1U << 4U)  /* Error State Indicator */

/**
 * @brief Parsed protocol packet structure
 */
typedef struct {
    uint8_t  cmd;
    uint8_t  seq;
    uint16_t len;
    uint8_t  payload[PROTO_MAX_PAYLOAD_SIZE];
    uint16_t crc;
} proto_packet_t;

/**
 * @brief Compute CRC16-CCITT (polynomial 0x1021, init 0xFFFF)
 * @param data Pointer to input data
 * @param length Length of data in bytes
 * @return 16-bit CRC checksum
 */
uint16_t proto_crc16(const uint8_t *data, size_t length);

/**
 * @brief Feed a single received byte into the protocol parser state machine
 * @param byte Incoming byte from serial stream
 * @param out_packet Pointer to packet struct to populate when a full packet is decoded
 * @return true if a complete and valid packet has been assembled, false otherwise
 */
bool proto_parse_byte(uint8_t byte, proto_packet_t *out_packet);

/**
 * @brief Reset the protocol parser state machine
 */
void proto_parse_reset(void);

/**
 * @brief Serialize an acknowledgment or error response into a transmission buffer
 * @param cmd Original command ID
 * @param seq Sequence number
 * @param status Status code (PROTO_STATUS_OK, etc.)
 * @param out_buf Buffer to store serialized bytes (minimum 10 bytes)
 * @param out_len Pointer to store total written length
 * @return ESP_OK on success
 */
esp_err_t proto_serialize_ack(uint8_t cmd, uint8_t seq, uint8_t status, uint8_t *out_buf, size_t *out_len);

/**
 * @brief Serialize a ping response with system telemetry
 * @param seq Sequence number
 * @param current_mode Current MCP2517FD operational mode
 * @param nominal_bps Current nominal bitrate in bps
 * @param data_bps Current data bitrate in bps
 * @param out_buf Buffer to store serialized bytes (minimum 26 bytes)
 * @param out_len Pointer to store total written length
 * @return ESP_OK on success
 */
esp_err_t proto_serialize_ping_rsp(uint8_t seq, uint8_t current_mode, uint32_t nominal_bps, uint32_t data_bps,
                                   uint8_t *out_buf, size_t *out_len);

/**
 * @brief Serialize a received CAN / CAN FD frame with microsecond timestamp
 * @param frame Pointer to received frame
 * @param timestamp_us Microsecond timestamp from esp_timer_get_time()
 * @param out_buf Buffer to store serialized bytes (minimum 88 bytes)
 * @param out_len Pointer to store total written length
 * @return ESP_OK on success
 */
esp_err_t proto_serialize_rx_frame(const canfd_frame_t *frame, uint64_t timestamp_us,
                                   uint8_t *out_buf, size_t *out_len);

/**
 * @brief Serialize a transmit confirmation notification
 * @param frame_id Transmitted CAN ID
 * @param seq Original sequence number
 * @param timestamp_us Transmit timestamp
 * @param status Transmit result status
 * @param out_buf Buffer to store serialized bytes (minimum 22 bytes)
 * @param out_len Pointer to store total written length
 * @return ESP_OK on success
 */
esp_err_t proto_serialize_tx_confirm(uint32_t frame_id, uint8_t seq, uint64_t timestamp_us, uint8_t status,
                                     uint8_t *out_buf, size_t *out_len);

/**
 * @brief Serialize bus diagnostics and statistics
 * @param diag Pointer to MCP2517FD diagnostics struct
 * @param rx_overflow_count Total RX overflow occurrences
 * @param rx_count Total frames received
 * @param tx_count Total frames transmitted
 * @param out_buf Buffer to store serialized bytes (minimum 32 bytes)
 * @param out_len Pointer to store total written length
 * @return ESP_OK on success
 */
esp_err_t proto_serialize_diag(const mcp2517fd_diag_t *diag, uint32_t rx_overflow_count,
                               uint32_t rx_count, uint32_t tx_count,
                               uint8_t *out_buf, size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* USB_PROTOCOL_H_ */
