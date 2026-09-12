#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;         /**< CAN Identifier (11-bit standard or 29-bit extended) */
    uint8_t  len;        /**< Payload length in bytes (0..64) */
    uint8_t  data[64];   /**< Payload data bytes */
    bool     is_extended;/**< true if 29-bit extended ID */
    bool     is_fd;      /**< true if CAN FD frame */
    bool     is_brs;     /**< true if Bit Rate Switch enabled */
    bool     is_rtr;     /**< true if Remote Transmission Request */
    uint8_t  filter_idx; /**< Matching filter index */
} canfd_frame_t;

typedef enum {
    MCP2517FD_OSC_40MHZ = 0U,
    MCP2517FD_OSC_20MHZ = 1U
} mcp2517fd_osc_t;

typedef struct {
    uint8_t  tec;
    uint8_t  rec;
    uint32_t trec_raw;
    uint32_t bdiag0_raw;
    uint32_t bdiag1_raw;
    bool     txbo;       /**< Bus-Off state */
    bool     txbp;       /**< TX Bus-Passive */
    bool     rxbp;       /**< RX Bus-Passive */
    bool     txwarn;     /**< TX Warning */
    bool     rxwarn;     /**< RX Warning */
    bool     nbit0_err;  /**< Nominal Bit 0 error: TX=0, RX=1 (transceiver didn't loop back) */
    bool     nbit1_err;  /**< Nominal Bit 1 error: TX=1, RX=0 */
    bool     nack_err;   /**< No ACK bit received from bus */
    bool     nstuff_err; /**< Stuff error (check baudrate / clock) */
    bool     ncrc_err;   /**< CRC error */
    bool     dbit0_err;  /**< Data Bit 0 error */
    bool     dbit1_err;  /**< Data Bit 1 error */
} mcp2517fd_diag_t;

typedef struct {
    gpio_num_t cs_io;           /**< GPIO number for nCS (e.g. GPIO_NUM_10) */
    gpio_num_t mosi_io;         /**< GPIO number for SDI/MOSI (e.g. GPIO_NUM_11) */
    gpio_num_t sck_io;          /**< GPIO number for SCK (e.g. GPIO_NUM_12) */
    gpio_num_t miso_io;         /**< GPIO number for SDO/MISO (e.g. GPIO_NUM_13) */
    gpio_num_t int_io;          /**< GPIO number for INT (e.g. GPIO_NUM_14) */
    uint32_t   spi_speed_hz;    /**< SPI Clock frequency in Hz (e.g. 20000000U) */
    mcp2517fd_osc_t osc;        /**< Oscillator frequency on board (40MHz or 20MHz) */
    uint32_t   nominal_bitrate; /**< Nominal/Arbitration bitrate in bps (e.g. 500000U) */
    uint32_t   data_bitrate;    /**< Data bitrate in bps (e.g. 2000000U) */
} mcp2517fd_config_t;

/**
 * @brief Initialize SPI bus, GPIOs, and MCP2517FD CAN FD controller
 * @param config Pointer to configuration struct
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mcp2517fd_init(const mcp2517fd_config_t *config);

/**
 * @brief Run an internal loopback self-test verifying SPI, RAM, FIFOs, and filters
 * @return ESP_OK if self-test passed, error code otherwise
 */
esp_err_t mcp2517fd_self_test(void);

/**
 * @brief Check if at least one message is waiting in the hardware RX FIFO
 * @return true if message is available, false otherwise
 */
bool mcp2517fd_has_rx_message(void);

/**
 * @brief Read one CAN/CAN FD frame from the MCP2517FD hardware RX FIFO
 * @param frame Pointer to frame structure to populate
 * @return ESP_OK if frame successfully read, ESP_ERR_NOT_FOUND if FIFO empty, error otherwise
 */
esp_err_t mcp2517fd_receive_frame(canfd_frame_t *frame);

/**
 * @brief Transmit a CAN or CAN FD frame using hardware TX FIFO
 * @param frame Pointer to frame structure to transmit
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if TX FIFO full, error code otherwise
 */
esp_err_t mcp2517fd_transmit_frame(const canfd_frame_t *frame);

/**
 * @brief Read current hardware RX FIFO overflow count
 * @return Number of RX FIFO overflows detected
 */
uint32_t mcp2517fd_get_rx_overflow_count(void);

/**
 * @brief Read comprehensive CAN bus error counters and diagnostic flags
 * @param diag Pointer to store diagnostics
 * @return ESP_OK on success
 */
esp_err_t mcp2517fd_get_diag(mcp2517fd_diag_t *diag);

#ifdef __cplusplus
}
#endif
