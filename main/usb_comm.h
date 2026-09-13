#ifndef USB_COMM_H_
#define USB_COMM_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize USB communication interface (USB-Serial-JTAG + UART fallback).
 *
 * Configures high-throughput hardware ring buffers (8KB TX, 4KB RX) for zero dropped
 * packets during high-speed CAN FD streaming.
 *
 * @return ESP_OK on success, or error code on failure.
 */
esp_err_t usb_comm_init(void);

/**
 * @brief Check if USB host is connected.
 * @return true if USB connection is active.
 */
bool usb_comm_is_connected(void);

/**
 * @brief Transmit raw bytes over the active USB/serial link.
 *
 * Copies bytes into the high-speed driver TX ring buffer. Returns immediately without
 * blocking if sufficient buffer space is available.
 *
 * @param data Pointer to buffer to transmit
 * @param len Number of bytes to transmit
 * @return Number of bytes successfully queued/written.
 */
int usb_comm_write(const uint8_t *data, size_t len);

/**
 * @brief Read incoming bytes from the USB/serial link.
 *
 * Non-blocking read from RX ring buffer.
 *
 * @param buf Pointer to buffer to receive data
 * @param max_len Maximum bytes to read
 * @return Number of bytes read, or 0 if no data available.
 */
int usb_comm_read(uint8_t *buf, size_t max_len);

/**
 * @brief Read incoming bytes from the USB/serial link with timeout.
 *
 * @param buf Pointer to buffer to receive data
 * @param max_len Maximum bytes to read
 * @param timeout_ms Maximum time in milliseconds to wait for data
 * @return Number of bytes read, or 0 if no data available within timeout.
 */
int usb_comm_read_timeout(uint8_t *buf, size_t max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* USB_COMM_H_ */
