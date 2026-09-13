#include "esp_log.h"
#include "mcp2517fd.h"
#include "can_tasks.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    /* Silence all logging to eliminate console output latency and CPU tick consumption */
    (void)esp_log_level_set("*", ESP_LOG_NONE);

    /* 2. Configure and initialize MCP2517FD Controller */
    mcp2517fd_config_t config;
    mcp2517fd_get_default_config(&config);

    esp_err_t err = mcp2517fd_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCP2517FD initialization failed: %s! Check wiring and power.", esp_err_to_name(err));
        return;
    }

    /* 3. Initialize Interrupt and Launch CAN Tasks (Core 1 RX, Core 0 Monitor) */
    err = can_tasks_init(config.int_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN tasks: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "System operational. Waiting for CAN FD frames...");
}
