#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mcp2517fd.h"
#include "can_tasks.h"
#include "usb_comm.h"
#include "usb_protocol.h"
#include "sdkconfig.h"

static const char *TAG = "CAN_TASKS";

#ifndef CONFIG_CANFD_MONITOR_INTERVAL_MS
#define CONFIG_CANFD_MONITOR_INTERVAL_MS 1000U
#endif

#ifndef CONFIG_CANFD_ENABLE_HEARTBEAT
#define CONFIG_CANFD_ENABLE_HEARTBEAT 0
#endif

#ifndef CONFIG_CANFD_HEARTBEAT_ID
#define CONFIG_CANFD_HEARTBEAT_ID 0x700U
#endif

typedef struct {
    canfd_frame_t frame;
    uint8_t seq;
} can_tx_item_t;

static TaskHandle_t s_rx_task_handle = NULL;
static TaskHandle_t s_mon_task_handle = NULL;
static TaskHandle_t s_usb_task_handle = NULL;
static QueueHandle_t s_tx_queue = NULL;
static uint32_t s_rx_success_count = 0U;
static uint32_t s_tx_success_count = 0U;

static void IRAM_ATTR mcp2517fd_gpio_isr(void *arg)
{
    (void)arg;
    BaseType_t high_task_woken = pdFALSE;
    if (s_rx_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_rx_task_handle, &high_task_woken);
        portYIELD_FROM_ISR(high_task_woken);
    }
}

static void canfd_rx_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "CAN FD high-speed RX/TX task running on Core %d (Priority %u)",
             xPortGetCoreID(), (unsigned int)uxTaskPriorityGet(NULL));

    canfd_frame_t frame;
    (void)memset(&frame, 0, sizeof(frame));
    uint8_t usb_pkt_buf[96];
    uint8_t tx_conf_buf[32];
    uint8_t usb_batch_buf[1024];
    size_t batch_len = 0U;
    int64_t last_idle_yield_us = esp_timer_get_time();

    for (;;) {
        /* Wait for interrupt notification from GPIO ISR or from TX enqueue.
         * A fallback timeout of 10ms (1 tick) allows CPU 1 to yield to IDLE1 when idle.
         */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));

        /* 1. Drain ALL available messages from hardware FIFO until COMPLETELY EMPTY.
         * The hardware INT pin is active LOW and stays LOW until the FIFO is empty.
         * Draining until empty ensures the INT pin returns HIGH, allowing new falling edges to fire.
         */
        while (mcp2517fd_receive_frame(&frame) == ESP_OK) {
            uint64_t now_us = (uint64_t)esp_timer_get_time();
            s_rx_success_count++;

            /* Stream frame over USB protocol with microsecond timestamp */
            size_t usb_len = 0U;
            if (proto_serialize_rx_frame(&frame, now_us, usb_pkt_buf, &usb_len) == ESP_OK) {
                if ((batch_len + usb_len) > sizeof(usb_batch_buf)) {
                    (void)usb_comm_write(usb_batch_buf, batch_len);
                    batch_len = 0U;
                }
                (void)memcpy(&usb_batch_buf[batch_len], usb_pkt_buf, usb_len);
                batch_len += usb_len;

                /* Flush in 256-byte chunks (4 full 64-byte USB packets) for maximum wire throughput */
                if (batch_len >= 256U) {
                    (void)usb_comm_write(usb_batch_buf, batch_len);
                    batch_len = 0U;
                }
            }
        }

        /* Flush remaining frames immediately so host receives all packets with zero delay */
        if (batch_len > 0U) {
            (void)usb_comm_write(usb_batch_buf, batch_len);
            batch_len = 0U;
        }

        /* 2. Process pending transmissions directly over SPI (sole owner of SPI bus) */
        if (s_tx_queue != NULL) {
            can_tx_item_t tx_item;
            while (xQueueReceive(s_tx_queue, &tx_item, 0) == pdPASS) {
                esp_err_t err = mcp2517fd_transmit_frame(&tx_item.frame);
                uint8_t status = (err == ESP_OK) ? PROTO_STATUS_OK : PROTO_STATUS_ERR_BUS_BUSY;
                if (err == ESP_OK) {
                    s_tx_success_count++;
                }
                uint64_t tx_time = (uint64_t)esp_timer_get_time();
                size_t conf_len = 0U;
                if (proto_serialize_tx_confirm(tx_item.frame.id, tx_item.seq, tx_time, status, tx_conf_buf, &conf_len) == ESP_OK) {
                    (void)usb_comm_write(tx_conf_buf, conf_len);
                }
            }
        }

        /* 3. Watchdog safety: If continuous flood persists without pause for >500ms,
         * yield 1 tick so IDLE1 can reset the task watchdog, only after draining FIFO.
         */
        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_idle_yield_us) > 500000LL) {
            vTaskDelay(1);
            last_idle_yield_us = esp_timer_get_time();
        }
    }
}

static void usb_cmd_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "USB command processor task running on Core %d", xPortGetCoreID());

    uint8_t rx_buf[256];
    proto_packet_t pkt;
    uint8_t tx_buf[128];
    size_t tx_len = 0U;

    for (;;) {
        int bytes_read = usb_comm_read_timeout(rx_buf, sizeof(rx_buf), 10U);
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; i++) {
                if (proto_parse_byte(rx_buf[i], &pkt)) {
                    /* Packet parsed and CRC16 verified */
                    switch (pkt.cmd) {
                    case PROTO_CMD_PING: {
                        uint32_t nom = 0U;
                        uint32_t dat = 0U;
                        mcp2517fd_get_bitrates(&nom, &dat);
                        uint8_t mode = mcp2517fd_get_mode();
                        if (proto_serialize_ping_rsp(pkt.seq, mode, nom, dat, tx_buf, &tx_len) == ESP_OK) {
                            (void)usb_comm_write(tx_buf, tx_len);
                        }
                        break;
                    }

                    case PROTO_CMD_SET_BITRATE: {
                        if (pkt.len >= 8U) {
                            uint32_t nom = (uint32_t)pkt.payload[0] |
                                           ((uint32_t)pkt.payload[1] << 8U) |
                                           ((uint32_t)pkt.payload[2] << 16U) |
                                           ((uint32_t)pkt.payload[3] << 24U);
                            uint32_t dat = (uint32_t)pkt.payload[4] |
                                           ((uint32_t)pkt.payload[5] << 8U) |
                                           ((uint32_t)pkt.payload[6] << 16U) |
                                           ((uint32_t)pkt.payload[7] << 24U);
                            esp_err_t err = mcp2517fd_set_bitrate(nom, dat);
                            uint8_t status = (err == ESP_OK) ? PROTO_STATUS_OK : PROTO_STATUS_ERR_GENERIC;
                            if (proto_serialize_ack(pkt.cmd, pkt.seq, status, tx_buf, &tx_len) == ESP_OK) {
                                (void)usb_comm_write(tx_buf, tx_len);
                            }
                        }
                        break;
                    }

                    case PROTO_CMD_SET_MODE: {
                        if (pkt.len >= 1U) {
                            uint8_t mode = pkt.payload[0];
                            esp_err_t err = mcp2517fd_set_mode(mode);
                            uint8_t status = (err == ESP_OK) ? PROTO_STATUS_OK : PROTO_STATUS_ERR_GENERIC;
                            if (proto_serialize_ack(pkt.cmd, pkt.seq, status, tx_buf, &tx_len) == ESP_OK) {
                                (void)usb_comm_write(tx_buf, tx_len);
                            }
                        }
                        break;
                    }

                    case PROTO_CMD_SET_FILTER: {
                        if (pkt.len >= 10U) {
                            uint8_t f_idx = pkt.payload[0];
                            bool is_ext = (pkt.payload[1] != 0U);
                            uint32_t fid = (uint32_t)pkt.payload[2] |
                                           ((uint32_t)pkt.payload[3] << 8U) |
                                           ((uint32_t)pkt.payload[4] << 16U) |
                                           ((uint32_t)pkt.payload[5] << 24U);
                            uint32_t mask = (uint32_t)pkt.payload[6] |
                                            ((uint32_t)pkt.payload[7] << 8U) |
                                            ((uint32_t)pkt.payload[8] << 16U) |
                                            ((uint32_t)pkt.payload[9] << 24U);
                            esp_err_t err = mcp2517fd_set_filter(f_idx, fid, mask, is_ext);
                            uint8_t status = (err == ESP_OK) ? PROTO_STATUS_OK : PROTO_STATUS_ERR_GENERIC;
                            if (proto_serialize_ack(pkt.cmd, pkt.seq, status, tx_buf, &tx_len) == ESP_OK) {
                                (void)usb_comm_write(tx_buf, tx_len);
                            }
                        }
                        break;
                    }

                    case PROTO_CMD_TX_FRAME: {
                        if (pkt.len >= 6U) {
                            can_tx_item_t tx_item;
                            (void)memset(&tx_item, 0, sizeof(tx_item));
                            tx_item.frame.id = (uint32_t)pkt.payload[0] |
                                               ((uint32_t)pkt.payload[1] << 8U) |
                                               ((uint32_t)pkt.payload[2] << 16U) |
                                               ((uint32_t)pkt.payload[3] << 24U);
                            uint8_t flags = pkt.payload[4];
                            tx_item.frame.is_extended = ((flags & PROTO_FRAME_FLAG_EXT) != 0U);
                            tx_item.frame.is_fd = ((flags & PROTO_FRAME_FLAG_FD) != 0U);
                            tx_item.frame.is_brs = ((flags & PROTO_FRAME_FLAG_BRS) != 0U);
                            tx_item.frame.is_rtr = ((flags & PROTO_FRAME_FLAG_RTR) != 0U);
                            uint8_t len = pkt.payload[5];
                            if (len > 64U) {
                                len = 64U;
                            }
                            tx_item.frame.len = len;
                            if ((len > 0U) && (pkt.len >= (6U + (uint16_t)len))) {
                                (void)memcpy(tx_item.frame.data, &pkt.payload[6], len);
                            }
                            tx_item.seq = pkt.seq;

                            /* Non-blocking enqueue to dedicated TX queue and wake Core 1 immediately */
                            BaseType_t q_res = pdFAIL;
                            if (s_tx_queue != NULL) {
                                q_res = xQueueSend(s_tx_queue, &tx_item, 0);
                            }
                            if (q_res == pdPASS) {
                                if (s_rx_task_handle != NULL) {
                                    xTaskNotifyGive(s_rx_task_handle);
                                }
                            }
                            uint8_t status = (q_res == pdPASS) ? PROTO_STATUS_OK : PROTO_STATUS_ERR_BUS_BUSY;
                            if (proto_serialize_ack(pkt.cmd, pkt.seq, status, tx_buf, &tx_len) == ESP_OK) {
                                (void)usb_comm_write(tx_buf, tx_len);
                            }
                        }
                        break;
                    }

                    case PROTO_CMD_BUS_CONTROL: {
                        if (pkt.len >= 1U) {
                            bool bus_on = (pkt.payload[0] != 0U);
                            esp_err_t err = mcp2517fd_bus_control(bus_on);
                            uint8_t status = (err == ESP_OK) ? PROTO_STATUS_OK : PROTO_STATUS_ERR_GENERIC;
                            if (proto_serialize_ack(pkt.cmd, pkt.seq, status, tx_buf, &tx_len) == ESP_OK) {
                                (void)usb_comm_write(tx_buf, tx_len);
                            }
                        }
                        break;
                    }

                    case PROTO_CMD_GET_DIAG: {
                        mcp2517fd_diag_t diag;
                        (void)memset(&diag, 0, sizeof(diag));
                        (void)mcp2517fd_get_diag(&diag);
                        if (proto_serialize_diag(&diag, mcp2517fd_get_rx_overflow_count(),
                                                 s_rx_success_count, s_tx_success_count,
                                                 tx_buf, &tx_len) == ESP_OK) {
                            (void)usb_comm_write(tx_buf, tx_len);
                        }
                        break;
                    }

                    case PROTO_CMD_RESET_STATS: {
                        s_rx_success_count = 0U;
                        s_tx_success_count = 0U;
                        if (proto_serialize_ack(pkt.cmd, pkt.seq, PROTO_STATUS_OK, tx_buf, &tx_len) == ESP_OK) {
                            (void)usb_comm_write(tx_buf, tx_len);
                        }
                        break;
                    }

                    default:
                        if (proto_serialize_ack(pkt.cmd, pkt.seq, PROTO_STATUS_ERR_INVALID_CMD, tx_buf, &tx_len) == ESP_OK) {
                            (void)usb_comm_write(tx_buf, tx_len);
                        }
                        break;
                    }
                }
            }
        }
    }
}

static void canfd_monitor_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "CAN FD monitor task running on Core %d", xPortGetCoreID());

    mcp2517fd_diag_t diag;
    (void)memset(&diag, 0, sizeof(diag));
    uint8_t diag_buf[32];
    size_t diag_len = 0U;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)CONFIG_CANFD_MONITOR_INTERVAL_MS));

        (void)mcp2517fd_get_diag(&diag);
        /* Broadcast periodic bus diagnostics over USB without slow console printing */
        if (proto_serialize_diag(&diag, mcp2517fd_get_rx_overflow_count(),
                                 s_rx_success_count, s_tx_success_count,
                                 diag_buf, &diag_len) == ESP_OK) {
            (void)usb_comm_write(diag_buf, diag_len);
        }
    }
}

esp_err_t can_tasks_init(gpio_num_t int_pin)
{
    /* 1. Initialize USB communication interface */
    esp_err_t err = usb_comm_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB comm init returned %s (continuing)", esp_err_to_name(err));
    }

    /* 2. Create high-capacity FreeRTOS TX queue (128 entries) for burst/multi-frame transmission */
    if (s_tx_queue == NULL) {
        s_tx_queue = xQueueCreate(128U, sizeof(can_tx_item_t));
        if (s_tx_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create TX queue");
            return ESP_ERR_NO_MEM;
        }
    }

    /* 3. Configure GPIO INT pin with pull-up and falling-edge interrupt */
    gpio_config_t io_conf;
    (void)memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = (1ULL << (uint32_t)int_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;

    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT GPIO %d: %s", (int)int_pin, esp_err_to_name(err));
        return err;
    }

    /* 4. Install GPIO ISR service and attach handler */
    err = gpio_install_isr_service(0);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(int_pin, mcp2517fd_gpio_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(err));
        return err;
    }

    /* 5. Create high-priority RX/TX task pinned to CPU Core 1 (Priority 10) */
    BaseType_t rx_created = xTaskCreatePinnedToCore(
        canfd_rx_task,
        "canfd_rx_task",
        4096U,
        NULL,
        10U,
        &s_rx_task_handle,
        1
    );
    if (rx_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create canfd_rx_task");
        return ESP_FAIL;
    }

    /* 6. Create USB command processing task pinned to CPU Core 0 (Priority 9) */
    BaseType_t usb_created = xTaskCreatePinnedToCore(
        usb_cmd_task,
        "usb_cmd_task",
        4096U,
        NULL,
        9U,
        &s_usb_task_handle,
        0
    );
    if (usb_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_cmd_task");
        return ESP_FAIL;
    }

    /* 7. Create periodic monitor task on CPU Core 0 (Priority 4) */
    BaseType_t mon_created = xTaskCreatePinnedToCore(
        canfd_monitor_task,
        "canfd_mon_task",
        4096U,
        NULL,
        4U,
        &s_mon_task_handle,
        0
    );
    if (mon_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create canfd_mon_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CAN tasks started: RX/TX Core 1 (Prio 10), USB Cmd Core 0 (Prio 9), Monitor Core 0 (Prio 4)");
    return ESP_OK;
}

void can_tasks_stop(void)
{
    if (s_rx_task_handle != NULL) {
        vTaskDelete(s_rx_task_handle);
        s_rx_task_handle = NULL;
    }
    if (s_usb_task_handle != NULL) {
        vTaskDelete(s_usb_task_handle);
        s_usb_task_handle = NULL;
    }
    if (s_mon_task_handle != NULL) {
        vTaskDelete(s_mon_task_handle);
        s_mon_task_handle = NULL;
    }
    if (s_tx_queue != NULL) {
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
    }
}
