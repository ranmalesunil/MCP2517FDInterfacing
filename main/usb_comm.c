#include "usb_comm.h"
#include <string.h>
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "USB_COMM";
static bool s_initialized = false;

esp_err_t usb_comm_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* 1. Initialize USB-Serial-JTAG with high-capacity 16KB TX and 8KB RX ring buffers */
    usb_serial_jtag_driver_config_t usj_cfg = {
        .tx_buffer_size = 16384U,
        .rx_buffer_size = 8192U
    };

    esp_err_t err = usb_serial_jtag_driver_install(&usj_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB-Serial-JTAG driver install returned %s (continuing with UART)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "USB-Serial-JTAG driver initialized (TX: %u B, RX: %u B)",
                 (unsigned int)usj_cfg.tx_buffer_size, (unsigned int)usj_cfg.rx_buffer_size);
    }

    /* 2. Configure UART0 for dual-port fallback only if not already installed */
    if (!uart_is_driver_installed(UART_NUM_0)) {
        uart_config_t uart_cfg = {
            .baud_rate = 460800,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        (void)uart_param_config(UART_NUM_0, &uart_cfg);
        (void)uart_driver_install(UART_NUM_0, 4096, 4096, 0, NULL, 0);
        ESP_LOGI(TAG, "UART0 driver initialized for dual-port fallback");
    }

    s_initialized = true;
    return ESP_OK;
}

bool usb_comm_is_connected(void)
{
    return usb_serial_jtag_is_connected();
}

int usb_comm_write(const uint8_t *data, size_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0;
    }

    /* 1. High-Speed Path: If USB-Serial-JTAG is active (e.g. COM3), write all bytes atomically.
     * Use a brief retry loop (up to 3 retries, 2ms wait) so packets are NEVER truncated under heavy load.
     */
    if (usb_serial_jtag_is_connected()) {
        size_t written = 0U;
        uint32_t retries = 0U;
        while ((written < len) && (retries < 3U)) {
            int ret = usb_serial_jtag_write_bytes(&data[written], len - written, pdMS_TO_TICKS(2U));
            if (ret > 0) {
                written += (size_t)ret;
            } else {
                retries++;
            }
        }
        return (int)written;
    }

    /* 2. Fallback Path: UART0 only if USB-Serial-JTAG is NOT connected.
     * Check buffer free space before writing so slow UART never blocks the CAN RX task.
     */
    if (uart_is_driver_installed(UART_NUM_0)) {
        size_t free_sz = 0U;
        (void)uart_get_tx_buffer_free_size(UART_NUM_0, &free_sz);
        if (free_sz >= len) {
            return uart_write_bytes(UART_NUM_0, (const char *)data, len);
        }
    }

    return 0;
}

int usb_comm_read_timeout(uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    if ((buf == NULL) || (max_len == 0U)) {
        return 0;
    }

    TickType_t ticks = (timeout_ms > 0U) ? pdMS_TO_TICKS(timeout_ms) : 0;
    if ((timeout_ms > 0U) && (ticks == 0)) {
        ticks = 1; /* Ensure at least 1 tick (10ms) wait */
    }

    /* 1. If USB-Serial-JTAG is active, read ONLY from USB-Serial-JTAG (zero UART latency) */
    if (usb_serial_jtag_is_connected()) {
        return usb_serial_jtag_read_bytes(buf, (uint32_t)max_len, ticks);
    }

    /* 2. Fallback to UART0 only if USB-Serial-JTAG is NOT connected */
    if (uart_is_driver_installed(UART_NUM_0)) {
        return uart_read_bytes(UART_NUM_0, buf, (uint32_t)max_len, ticks);
    }

    return 0;
}

int usb_comm_read(uint8_t *buf, size_t max_len)
{
    return usb_comm_read_timeout(buf, max_len, 0U);
}
