#ifndef MCP2517FD_H_
#define MCP2517FD_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Hardware Pin Configurations (from Kconfig with defaults)
 * ========================================================================= */
#ifndef CONFIG_MCP2517FD_PIN_CS
#define CONFIG_MCP2517FD_PIN_CS 10
#endif
#ifndef CONFIG_MCP2517FD_PIN_MOSI
#define CONFIG_MCP2517FD_PIN_MOSI 11
#endif
#ifndef CONFIG_MCP2517FD_PIN_SCK
#define CONFIG_MCP2517FD_PIN_SCK 12
#endif
#ifndef CONFIG_MCP2517FD_PIN_MISO
#define CONFIG_MCP2517FD_PIN_MISO 13
#endif
#ifndef CONFIG_MCP2517FD_PIN_INT
#define CONFIG_MCP2517FD_PIN_INT 14
#endif
#ifndef CONFIG_MCP2517FD_NOMINAL_BITRATE
#define CONFIG_MCP2517FD_NOMINAL_BITRATE 500000U
#endif
#ifndef CONFIG_MCP2517FD_DATA_BITRATE
#define CONFIG_MCP2517FD_DATA_BITRATE 2000000U
#endif

#define MCP2517FD_PIN_CS        ((gpio_num_t)CONFIG_MCP2517FD_PIN_CS)
#define MCP2517FD_PIN_MOSI      ((gpio_num_t)CONFIG_MCP2517FD_PIN_MOSI)
#define MCP2517FD_PIN_SCK       ((gpio_num_t)CONFIG_MCP2517FD_PIN_SCK)
#define MCP2517FD_PIN_MISO      ((gpio_num_t)CONFIG_MCP2517FD_PIN_MISO)
#define MCP2517FD_PIN_INT       ((gpio_num_t)CONFIG_MCP2517FD_PIN_INT)
#define MCP2517FD_PIN_STBY0     ((gpio_num_t)GPIO_NUM_15)
#define MCP2517FD_PIN_STBY1     ((gpio_num_t)GPIO_NUM_16)

#define MCP2517FD_NOMINAL_RATE  ((uint32_t)CONFIG_MCP2517FD_NOMINAL_BITRATE)
#define MCP2517FD_DATA_RATE     ((uint32_t)CONFIG_MCP2517FD_DATA_BITRATE)

/* =========================================================================
 * Data Structures
 * ========================================================================= */
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

/* =========================================================================
 * Driver API Functions
 * ========================================================================= */

/**
 * @brief Populate a configuration structure with default pins and settings
 * @param config Pointer to configuration struct to populate
 */
void mcp2517fd_get_default_config(mcp2517fd_config_t *config);

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

/**
 * @brief Dynamically set CAN nominal and data bitrates
 * @param nominal_bps Nominal/arbitration bitrate in bps (e.g. 500000, 250000, 125000, 1000000)
 * @param data_bps Data bitrate in bps (e.g. 2000000, 4000000, 5000000, or 0/same for CAN 2.0)
 * @return ESP_OK on success, or error code
 */
esp_err_t mcp2517fd_set_bitrate(uint32_t nominal_bps, uint32_t data_bps);

/**
 * @brief Dynamically change MCP2517FD operational mode
 * @param mode Target mode (e.g. MCP2517FD_MODE_NORMAL_CANFD, MCP2517FD_MODE_LISTEN_ONLY, etc.)
 * @return ESP_OK on success, or error code
 */
esp_err_t mcp2517fd_set_mode(uint8_t mode);

/**
 * @brief Dynamically configure an acceptance filter and mask
 * @param filter_idx Filter index (0..31)
 * @param filter_id Filter CAN ID
 * @param mask Mask value (0 = accept all, 0x7FF/0x1FFFFFFF = exact match)
 * @param is_ext true if matching 29-bit extended ID
 * @return ESP_OK on success, or error code
 */
esp_err_t mcp2517fd_set_filter(uint8_t filter_idx, uint32_t filter_id, uint32_t mask, bool is_ext);

/**
 * @brief Enable or disable CAN bus activity (Bus-On / Bus-Off)
 * @param enable true for Bus-On (Normal CAN FD), false for Bus-Off (Configuration mode)
 * @return ESP_OK on success
 */
esp_err_t mcp2517fd_bus_control(bool enable);

/**
 * @brief Query current operational mode
 * @return Current mode code
 */
uint8_t mcp2517fd_get_mode(void);

/**
 * @brief Query currently configured bitrates
 * @param nominal_bps Pointer to store nominal bitrate
 * @param data_bps Pointer to store data bitrate
 */
void mcp2517fd_get_bitrates(uint32_t *nominal_bps, uint32_t *data_bps);

#ifdef __cplusplus
}
#endif

#endif /* MCP2517FD_H_ */
