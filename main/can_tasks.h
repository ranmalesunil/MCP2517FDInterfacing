#ifndef CAN_TASKS_H_
#define CAN_TASKS_H_

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize interrupt and create CAN FD FreeRTOS tasks (RX and Monitor/Heartbeat).
 *
 * Configures the MCP2517FD INT pin with falling-edge interrupt, attaches the ISR handler,
 * and launches:
 *   - canfd_rx_task on CPU Core 1 (Priority 10)
 *   - canfd_monitor_task on CPU Core 0 (Priority 4)
 *
 * @param int_pin GPIO number connected to MCP2517FD INT pin.
 * @return ESP_OK on success, or error code on failure.
 */
esp_err_t can_tasks_init(gpio_num_t int_pin);

/**
 * @brief Stop and delete CAN FD FreeRTOS tasks.
 */
void can_tasks_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TASKS_H_ */
