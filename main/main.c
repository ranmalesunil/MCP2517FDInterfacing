#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mcp2517fd.h"
#include "can_tracker.h"
#include "sdkconfig.h"

static const char *TAG = "MAIN";

/* Fallback configurations if not defined in Kconfig */
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
#ifndef CONFIG_CANFD_MONITOR_INTERVAL_MS
#define CONFIG_CANFD_MONITOR_INTERVAL_MS 1000U
#endif
#ifndef CONFIG_CANFD_ENABLE_HEARTBEAT
#define CONFIG_CANFD_ENABLE_HEARTBEAT 1
#endif
#ifndef CONFIG_CANFD_HEARTBEAT_ID
#define CONFIG_CANFD_HEARTBEAT_ID 0x700U
#endif

#define MCP2517FD_PIN_CS    ((gpio_num_t)CONFIG_MCP2517FD_PIN_CS)
#define MCP2517FD_PIN_MOSI  ((gpio_num_t)CONFIG_MCP2517FD_PIN_MOSI)
#define MCP2517FD_PIN_SCK   ((gpio_num_t)CONFIG_MCP2517FD_PIN_SCK)
#define MCP2517FD_PIN_MISO  ((gpio_num_t)CONFIG_MCP2517FD_PIN_MISO)
#define MCP2517FD_PIN_INT   ((gpio_num_t)CONFIG_MCP2517FD_PIN_INT)

static TaskHandle_t s_rx_task_handle = NULL;

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
    ESP_LOGI(TAG, "CAN FD high-speed RX task running on Core %d (Priority %u)",
             xPortGetCoreID(), (unsigned int)uxTaskPriorityGet(NULL));

    canfd_frame_t frame;
    (void)memset(&frame, 0, sizeof(frame));

    for (;;) {
        /* Wait for interrupt notification from GPIO ISR.
         * A fallback timeout of 10ms (1 FreeRTOS tick at CONFIG_FREERTOS_HZ=100) ensures
         * the task actually blocks and yields CPU 1 to the IDLE1 task, preventing Task Watchdog (TWDT) triggers.
         */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));

        /* Batch drain available messages directly from hardware FIFO (up to 32 per burst) */
        uint32_t drained = 0U;
        while ((drained < 32U) && (mcp2517fd_receive_frame(&frame) == ESP_OK)) {
            can_tracker_record_frame(&frame);
            drained++;
        }
    }
}

static void canfd_monitor_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "CAN FD monitor and heartbeat task running on Core %d", xPortGetCoreID());

    mcp2517fd_diag_t diag;
    (void)memset(&diag, 0, sizeof(diag));
    uint32_t heartbeat_counter = 0U;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)CONFIG_CANFD_MONITOR_INTERVAL_MS));

#if (CONFIG_CANFD_ENABLE_HEARTBEAT != 0)
        /* Send periodic heartbeat CAN FD frame */
        canfd_frame_t hb_frame;
        (void)memset(&hb_frame, 0, sizeof(hb_frame));
        hb_frame.id = (uint32_t)CONFIG_CANFD_HEARTBEAT_ID;
        hb_frame.len = 8U;
        hb_frame.is_extended = false;
        hb_frame.is_rtr = false;
#if defined(CONFIG_CANFD_HEARTBEAT_FD_BRS)
        hb_frame.is_fd = true;
        hb_frame.is_brs = true;
#elif defined(CONFIG_CANFD_HEARTBEAT_FD_NO_BRS)
        hb_frame.is_fd = true;
        hb_frame.is_brs = false;
#else
        /* Default: Classic CAN 2.0 (Solid 500 kbps throughout, No FD, No BRS) */
        hb_frame.is_fd = false;
        hb_frame.is_brs = false;
#endif

        uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000LL);
        (void)memcpy(&hb_frame.data[0], &uptime_sec, 4U);
        (void)memcpy(&hb_frame.data[4], &heartbeat_counter, 4U);

        esp_err_t tx_err = mcp2517fd_transmit_frame(&hb_frame);
        if (tx_err == ESP_OK) {
            heartbeat_counter++;
        } else if (tx_err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Heartbeat TX pending in hardware buffer (waiting for bus ACK from PCAN)");
        } else {
            ESP_LOGE(TAG, "Heartbeat TX failed: %s", esp_err_to_name(tx_err));
        }
#endif

        (void)mcp2517fd_get_diag(&diag);
        can_tracker_print_summary(&diag);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  ESP32-S3 High-Speed CAN FD Monitor (MCP2517FD)  ");
    ESP_LOGI(TAG, "  CS:   GPIO%d (Pin 13)                          ", (int)MCP2517FD_PIN_CS);
    ESP_LOGI(TAG, "  MOSI: GPIO%d (Pin 11)                          ", (int)MCP2517FD_PIN_MOSI);
    ESP_LOGI(TAG, "  SCK:  GPIO%d (Pin 10)                          ", (int)MCP2517FD_PIN_SCK);
    ESP_LOGI(TAG, "  MISO: GPIO%d (Pin 12)                          ", (int)MCP2517FD_PIN_MISO);
    ESP_LOGI(TAG, "  INT:  GPIO%d (Pin 4)                           ", (int)MCP2517FD_PIN_INT);
    ESP_LOGI(TAG, "  GPIO15 & 16: Driven LOW (Active Standby disable)");
#if (CONFIG_CANFD_ENABLE_HEARTBEAT != 0)
    ESP_LOGI(TAG, "  Heartbeat: ENABLED (ID: 0x%03X, 1 msg/sec)      ", (unsigned int)CONFIG_CANFD_HEARTBEAT_ID);
#endif
    ESP_LOGI(TAG, "==================================================");

    /* Initialize Tracker Data Structures */
    can_tracker_init();

    /* Explicitly drive GPIO15 and GPIO16 LOW in case transceiver STBY is tied here */
    (void)gpio_reset_pin(GPIO_NUM_15);
    (void)gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(GPIO_NUM_15, 0);

    (void)gpio_reset_pin(GPIO_NUM_16);
    (void)gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(GPIO_NUM_16, 0);

    /* Configure MCP2517FD Controller */
    mcp2517fd_config_t config;
    (void)memset(&config, 0, sizeof(config));
    config.cs_io = MCP2517FD_PIN_CS;
    config.mosi_io = MCP2517FD_PIN_MOSI;
    config.sck_io = MCP2517FD_PIN_SCK;
    config.miso_io = MCP2517FD_PIN_MISO;
    config.int_io = MCP2517FD_PIN_INT;
    config.spi_speed_hz = 20000000U; /* 20 MHz full-speed SPI clock */
#ifdef CONFIG_MCP2517FD_OSC_20MHZ
    config.osc = MCP2517FD_OSC_20MHZ;
#else
    config.osc = MCP2517FD_OSC_40MHZ;
#endif
    config.nominal_bitrate = (uint32_t)CONFIG_MCP2517FD_NOMINAL_BITRATE;
    config.data_bitrate = (uint32_t)CONFIG_MCP2517FD_DATA_BITRATE;

    esp_err_t err = mcp2517fd_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCP2517FD initialization failed: %s! Check SPI wiring and power.", esp_err_to_name(err));
    } else {
        /* Configure GPIO 14 (INT pin) with pullup and falling-edge interrupt */
        gpio_config_t io_conf;
        (void)memset(&io_conf, 0, sizeof(io_conf));
        io_conf.pin_bit_mask = (1ULL << (uint32_t)MCP2517FD_PIN_INT);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_NEGEDGE;

        esp_err_t gerr = gpio_config(&io_conf);
        if (gerr != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure INT GPIO: %s", esp_err_to_name(gerr));
        }

        /* Install GPIO ISR service and attach handler */
        gerr = gpio_install_isr_service(0);
        if ((gerr != ESP_OK) && (gerr != ESP_ERR_INVALID_STATE)) {
            ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(gerr));
        }
        gerr = gpio_isr_handler_add(MCP2517FD_PIN_INT, mcp2517fd_gpio_isr, NULL);
        if (gerr != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(gerr));
        }

        /* Create high-priority RX task pinned to CPU Core 1 */
        (void)xTaskCreatePinnedToCore(canfd_rx_task, "canfd_rx_task", 4096U, NULL, 20U, &s_rx_task_handle, 1);

        /* Create periodic monitor task on CPU Core 0 */
        (void)xTaskCreatePinnedToCore(canfd_monitor_task, "canfd_mon_task", 4096U, NULL, 4U, NULL, 0);

        ESP_LOGI(TAG, "System operational. Waiting for CAN FD frames...");
    }
}
