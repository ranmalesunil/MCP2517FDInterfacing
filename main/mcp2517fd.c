#include "mcp2517fd.h"
#include "mcp2517fd_regs.h"
#include <string.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_rom_sys.h"

static const char *TAG = "MCP2517FD";

static spi_device_handle_t s_spi_dev = NULL;
static SemaphoreHandle_t s_spi_mutex = NULL;
static uint32_t s_rx_overflow_count = 0U;
static uint32_t s_current_nominal_bps = MCP2517FD_NOMINAL_RATE;
static uint32_t s_current_data_bps = MCP2517FD_DATA_RATE;
static uint8_t s_current_mode = MCP2517FD_MODE_NORMAL_CANFD;
static mcp2517fd_osc_t s_current_osc = MCP2517FD_OSC_40MHZ;
static const uint8_t s_dlc_to_len[16] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

static uint8_t len_to_dlc(uint8_t len)
{
    uint8_t dlc;
    if (len <= 8U) {
        dlc = len;
    } else if (len <= 12U) {
        dlc = 9U;
    } else if (len <= 16U) {
        dlc = 10U;
    } else if (len <= 20U) {
        dlc = 11U;
    } else if (len <= 24U) {
        dlc = 12U;
    } else if (len <= 32U) {
        dlc = 13U;
    } else if (len <= 48U) {
        dlc = 14U;
    } else {
        dlc = 15U;
    }
    return dlc;
}

/* Helper SPI register access functions */
static esp_err_t mcp2517fd_read_regs(uint16_t addr, uint8_t *data, size_t len)
{
    esp_err_t err = ESP_OK;
    uint8_t tx_buf[80] = {0U};
    uint8_t rx_buf[80] = {0U};

    if (data == NULL) {
        err = ESP_ERR_INVALID_ARG;
    } else if ((len + 2U) > sizeof(tx_buf)) {
        err = ESP_ERR_INVALID_SIZE;
    } else {
        uint16_t cmd = (uint16_t)((addr & 0x0FFFU) | MCP2517FD_CMD_READ);
        tx_buf[0] = (uint8_t)(cmd >> 8U);
        tx_buf[1] = (uint8_t)(cmd & 0xFFU);

        spi_transaction_t t;
        (void)memset(&t, 0, sizeof(t));
        t.length = (2U + len) * 8U;
        t.tx_buffer = tx_buf;
        t.rx_buffer = rx_buf;

        err = spi_device_polling_transmit(s_spi_dev, &t);
        if (err == ESP_OK) {
            (void)memcpy(data, &rx_buf[2], len);
        }
    }
    return err;
}

static esp_err_t mcp2517fd_write_regs(uint16_t addr, const uint8_t *data, size_t len)
{
    esp_err_t err = ESP_OK;
    uint8_t tx_buf[80] = {0U};

    if (data == NULL) {
        err = ESP_ERR_INVALID_ARG;
    } else if ((len + 2U) > sizeof(tx_buf)) {
        err = ESP_ERR_INVALID_SIZE;
    } else {
        uint16_t cmd = (uint16_t)((addr & 0x0FFFU) | MCP2517FD_CMD_WRITE);
        tx_buf[0] = (uint8_t)(cmd >> 8U);
        tx_buf[1] = (uint8_t)(cmd & 0xFFU);
        (void)memcpy(&tx_buf[2], data, len);

        spi_transaction_t t;
        (void)memset(&t, 0, sizeof(t));
        t.length = (2U + len) * 8U;
        t.tx_buffer = tx_buf;
        t.rx_buffer = NULL;

        err = spi_device_polling_transmit(s_spi_dev, &t);
    }
    return err;
}

static uint8_t mcp2517fd_read8(uint16_t addr)
{
    uint8_t val = 0U;
    (void)mcp2517fd_read_regs(addr, &val, 1U);
    return val;
}

static void mcp2517fd_write8(uint16_t addr, uint8_t val)
{
    (void)mcp2517fd_write_regs(addr, &val, 1U);
}

static uint32_t mcp2517fd_read32(uint16_t addr)
{
    uint32_t val = 0U;
    (void)mcp2517fd_read_regs(addr, (uint8_t *)&val, 4U);
    return val;
}

static void mcp2517fd_write32(uint16_t addr, uint32_t val)
{
    (void)mcp2517fd_write_regs(addr, (const uint8_t *)&val, 4U);
}

static esp_err_t mcp2517fd_cmd_reset(void)
{
    uint8_t tx_buf[2] = {0x00U, 0x00U};
    spi_transaction_t t;
    (void)memset(&t, 0, sizeof(t));
    t.length = 16U;
    t.tx_buffer = tx_buf;
    t.rx_buffer = NULL;
    return spi_device_polling_transmit(s_spi_dev, &t);
}

static esp_err_t mcp2517fd_request_mode(uint8_t mode)
{
    esp_err_t ret = ESP_OK;
    /* Read current operating mode OPMOD[2:0] in CiCON byte 2 (address 0x002) */
    uint8_t current_mode = (uint8_t)((mcp2517fd_read8((uint16_t)(MCP2517FD_REG_CiCON + 2U)) >> 5U) & 0x07U);
    if (current_mode != mode) {
        /* MCP2517FD hardware requirement: All mode transitions between operational modes
         * (e.g. from Internal Loopback to Normal mode) MUST pass through Configuration mode (Mode 4).
         */
        if ((current_mode != MCP2517FD_MODE_CONFIGURATION) && (mode != MCP2517FD_MODE_CONFIGURATION)) {
            /* Request Configuration mode with ABAT = 1 (Abort All Pending Transmissions) */
            mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiCON + 3U), (uint8_t)(MCP2517FD_MODE_CONFIGURATION | (1U << 3U)));
            bool in_cfg = false;
            for (uint32_t i = 0U; i < 50U; i++) {
                current_mode = (uint8_t)((mcp2517fd_read8((uint16_t)(MCP2517FD_REG_CiCON + 2U)) >> 5U) & 0x07U);
                if (current_mode == MCP2517FD_MODE_CONFIGURATION) {
                    in_cfg = true;
                    break;
                }
                esp_rom_delay_us(100);
            }
            if (!in_cfg) {
                ESP_LOGE(TAG, "Intermediate Configuration mode transition timeout! Actual: %u", (unsigned int)current_mode);
                ret = ESP_ERR_TIMEOUT;
            }
        }

        if (ret == ESP_OK) {
            uint8_t val = (uint8_t)(mode & 0x07U);
            if (mode == MCP2517FD_MODE_CONFIGURATION) {
                val |= (uint8_t)(1U << 3U); /* ABAT */
            }
            mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiCON + 3U), val);

            bool mode_achieved = false;
            for (uint32_t i = 0U; i < 50U; i++) {
                current_mode = (uint8_t)((mcp2517fd_read8((uint16_t)(MCP2517FD_REG_CiCON + 2U)) >> 5U) & 0x07U);
                if (current_mode == mode) {
                    ESP_LOGI(TAG, "Mode successfully changed to %u", (unsigned int)mode);
                    mode_achieved = true;
                    break;
                }
                esp_rom_delay_us(100);
            }
            if (!mode_achieved) {
                ESP_LOGE(TAG, "Mode change timeout! Requested: %u, Actual: %u", (unsigned int)mode, (unsigned int)current_mode);
                ret = ESP_ERR_TIMEOUT;
            }
        }
    }
    return ret;
}

void mcp2517fd_get_default_config(mcp2517fd_config_t *config)
{
    if (config != NULL) {
        (void)memset(config, 0, sizeof(mcp2517fd_config_t));
        config->cs_io = MCP2517FD_PIN_CS;
        config->mosi_io = MCP2517FD_PIN_MOSI;
        config->sck_io = MCP2517FD_PIN_SCK;
        config->miso_io = MCP2517FD_PIN_MISO;
        config->int_io = MCP2517FD_PIN_INT;
        config->spi_speed_hz = 20000000U;
#ifdef CONFIG_MCP2517FD_OSC_20MHZ
        config->osc = MCP2517FD_OSC_20MHZ;
#else
        config->osc = MCP2517FD_OSC_40MHZ;
#endif
        config->nominal_bitrate = MCP2517FD_NOMINAL_RATE;
        config->data_bitrate = MCP2517FD_DATA_RATE;
    }
}

esp_err_t mcp2517fd_init(const mcp2517fd_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_current_nominal_bps = config->nominal_bitrate;
    s_current_data_bps = config->data_bitrate;
    s_current_osc = config->osc;
    s_current_mode = MCP2517FD_MODE_NORMAL_CANFD;

    /* Drive optional transceiver standby pins LOW to disable standby */
    (void)gpio_reset_pin(MCP2517FD_PIN_STBY0);
    (void)gpio_set_direction(MCP2517FD_PIN_STBY0, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(MCP2517FD_PIN_STBY0, 0);

    (void)gpio_reset_pin(MCP2517FD_PIN_STBY1);
    (void)gpio_set_direction(MCP2517FD_PIN_STBY1, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(MCP2517FD_PIN_STBY1, 0);

    ESP_LOGI(TAG, "Initializing MCP2517FD SPI bus (MOSI=%d, MISO=%d, SCK=%d, CS=%d, INT=%d)...",
             (int)config->mosi_io, (int)config->miso_io, (int)config->sck_io, (int)config->cs_io, (int)config->int_io);

    spi_bus_config_t buscfg;
    (void)memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = (int)config->mosi_io;
    buscfg.miso_io_num = (int)config->miso_io;
    buscfg.sclk_io_num = (int)config->sck_io;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 128;

    if (s_spi_mutex == NULL) {
        s_spi_mutex = xSemaphoreCreateMutex();
    }

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE)) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    uint32_t speed = (config->spi_speed_hz > 0U) ? config->spi_speed_hz : 10000000U;
    spi_device_interface_config_t devcfg;
    (void)memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = (int)speed;
    devcfg.mode = 0;
    devcfg.spics_io_num = (int)config->cs_io;
    devcfg.queue_size = 1;

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 1. Issue Hardware Reset command upon startup */
    vTaskDelay(pdMS_TO_TICKS(10U));
    (void)mcp2517fd_cmd_reset();
    vTaskDelay(pdMS_TO_TICKS(15U));

    /* 2. Wait for chip to settle in Configuration mode (Mode 4) after reset */
    bool in_config = false;
    for (uint32_t i = 0U; i < 50U; i++) {
        uint8_t current_mode = (uint8_t)((mcp2517fd_read8((uint16_t)(MCP2517FD_REG_CiCON + 2U)) >> 5U) & 0x07U);
        if (current_mode == MCP2517FD_MODE_CONFIGURATION) {
            in_config = true;
            break;
        }
        esp_rom_delay_us(100);
    }
    if (!in_config) {
        ESP_LOGW(TAG, "Chip not in Configuration mode immediately after reset, attempting direct mode request...");
        mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiCON + 3U), (uint8_t)(MCP2517FD_MODE_CONFIGURATION | (1U << 3U)));
        vTaskDelay(pdMS_TO_TICKS(10U));
    }

    /* 3. Verify communication with RAM read/write check */
    const uint32_t test_patterns[3] = {0x55AA55AAU, 0x12345678U, 0xA5A5A5A5U};
    for (uint32_t i = 0U; i < 3U; i++) {
        mcp2517fd_write32(MCP2517FD_RAM_ADDR_START, test_patterns[i]);
        uint32_t readback = mcp2517fd_read32(MCP2517FD_RAM_ADDR_START);
        if (readback != test_patterns[i]) {
            if (readback == 0x00000000U) {
                ESP_LOGE(TAG, "SPI failure: Wrote 0x%08lX, read back ALL ZEROES (0x00000000)!", (unsigned long)test_patterns[i]);
                ESP_LOGE(TAG, "  -> GPIO 13 (MISO / SDO) is disconnected or loose");
                ESP_LOGE(TAG, "  -> OR 3.3V (VDD, Pin 14) / GND (VSS, Pin 7) is unpowered");
            } else if (readback == 0xFFFFFFFFU) {
                ESP_LOGE(TAG, "SPI failure: Wrote 0x%08lX, read back ALL ONES (0xFFFFFFFF)!", (unsigned long)test_patterns[i]);
                ESP_LOGE(TAG, "  -> GPIO 10 (CS, Pin 13) or GPIO 12 (SCK, Pin 10) is disconnected");
            } else {
                ESP_LOGE(TAG, "SPI communication failure: wrote 0x%08lX, read back 0x%08lX",
                         (unsigned long)test_patterns[i], (unsigned long)readback);
            }
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    ESP_LOGI(TAG, "SPI communication verified successfully");

    /* 4. Configure Oscillator */
    mcp2517fd_write8(MCP2517FD_REG_OSC, 0x00U); /* divide by 1, no PLL */
    vTaskDelay(pdMS_TO_TICKS(10U));

    uint8_t osc_val = mcp2517fd_read8(MCP2517FD_REG_OSC);
    uint8_t osc_sta = mcp2517fd_read8((uint16_t)(MCP2517FD_REG_OSC + 1U));
    ESP_LOGI(TAG, "OSC register: 0x%02X, Status: 0x%02X (SCLKRDY=%u, PLLRDY=%u, OSCRDY=%u)",
             osc_val, osc_sta, (unsigned int)((osc_sta >> 2U) & 1U), (unsigned int)(osc_sta & 1U), (unsigned int)((osc_sta >> 4U) & 1U));

    /* 5. Clear all 2048 bytes of RAM */
    uint8_t zero_buf[32] = {0U};
    for (uint16_t addr = MCP2517FD_RAM_ADDR_START; addr < (MCP2517FD_RAM_ADDR_START + MCP2517FD_RAM_SIZE); addr += (uint16_t)sizeof(zero_buf)) {
        (void)mcp2517fd_write_regs(addr, zero_buf, sizeof(zero_buf));
    }

    /* 6. Configure IOCON:
          - Byte 0 (0xE04): XSTBYEN = 1 (bit 6: auto wake-up CAN transceiver), TRIS0=0, TRIS1=0
          - Byte 1 (0xE05): LAT0 = 0, LAT1 = 0 (drive LOW 0V for normal mode)
          - Byte 3 (0xE07): INT pin active-low push-pull (bits [1:0] = 0b11)
    */
    mcp2517fd_write8(MCP2517FD_REG_IOCON, 0x40U);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_IOCON + 1U), 0x00U);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_IOCON + 3U), 0x03U);

    /* 7. Configure CiCON: PXEDIS = 1, ISOCRC = 1, RTXAT = 1 */
    mcp2517fd_write8(MCP2517FD_REG_CiCON, 0x60U);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiCON + 2U), 0x01U);

    /* 8. Configure Bit Timing matching PCAN parameters:
          Nominal: 500 kbps, 81.25% sample point (16 TQ: BRP=4, TSEG1=11, TSEG2=2, SJW=2)
          Data: 2000 kbps (2 Mbps), 80.0% sample point (20 TQ: BRP=0, TSEG1=14, TSEG2=3, SJW=3), Auto TDC
    */
    uint32_t nbtcfg;
    uint32_t dbtcfg;
    uint32_t tdc;

    if (config->osc == MCP2517FD_OSC_20MHZ) {
        /* 20 MHz Clock:
           Nominal 500k: 40 TQ (BRP=0, TSEG1=30, TSEG2=7, SJW=7) -> 80% sample point
           Data 2M: 10 TQ (BRP=0, TSEG1=6, TSEG2=1, SJW=1) -> 80% sample point
        */
        nbtcfg = (0UL << 24U) | (30UL << 16U) | (7UL << 8U) | 7UL;
        dbtcfg = (0UL << 24U) | (6UL << 16U) | (1UL << 8U) | 1UL;
        tdc = (1UL << 25U) | (1UL << 17U) | (7UL << 8U);
    } else {
        /* 40 MHz Clock:
           Nominal 500k: BRP=5 (8MHz clock, 16 TQ: TSEG1=11, TSEG2=2, SJW=2) -> 81.25% sample point (EXACT MATCH TO PCAN!)
           Data 2M: BRP=1 (40MHz clock, 20 TQ: TSEG1=14, TSEG2=3, SJW=3) -> 80.0% sample point (EXACT MATCH TO PCAN!)
        */
        nbtcfg = (4UL << 24U) | (11UL << 16U) | (2UL << 8U) | 2UL;
        dbtcfg = (0UL << 24U) | (14UL << 16U) | (3UL << 8U) | 3UL;
        tdc = (1UL << 25U) | (1UL << 17U) | (15UL << 8U); /* Auto TDC, Offset = 15 TQ = 375ns */
    }

    mcp2517fd_write32(MCP2517FD_REG_CiNBTCFG, nbtcfg);
    mcp2517fd_write32(MCP2517FD_REG_CiDBTCFG, dbtcfg);
    mcp2517fd_write32(MCP2517FD_REG_CiTDC, tdc);

    /* 9. Configure RX FIFO 1:
          - Payload size: 64 bytes (PLSIZE = 7)
          - FIFO size: 22 entries (FSIZE = 21) -> 22 * 72 bytes = 1584 bytes in RAM
          - RX FIFO (TXEN = 0)
          - Enable RX Not Empty Interrupt (TFNRFNIE = 1)
          - Enable Overflow Interrupt (RXOVIE = 1)
    */
    uint8_t rx_fifo_pl_size = (uint8_t)((MCP2517FD_PLSIZE_64 << 5U) | 21U);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(1U) + 3U), rx_fifo_pl_size);
    mcp2517fd_write8(MCP2517FD_REG_CiFIFOCON(1U), (uint8_t)(MCP2517FD_FIFOCON_TFNRFNIE | MCP2517FD_FIFOCON_RXOVIE));

    /* 10. Configure TX FIFO 2:
          - Payload size: 64 bytes (PLSIZE = 7)
          - FIFO size: 6 entries (FSIZE = 5) -> 6 * 72 bytes = 432 bytes in RAM
          - Total RAM used (FIFO1 + FIFO2) = 1584 + 432 = 2016 bytes (<= 2048)
          - TX FIFO (TXEN = 1)
    */
    uint8_t tx_fifo_pl_size = (uint8_t)((MCP2517FD_PLSIZE_64 << 5U) | 5U);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(2U) + 3U), tx_fifo_pl_size);
    /* Enable TX FIFO with highest transmit priority (TXPRI = 3, bits [6:5] = 0b11) */
    mcp2517fd_write8(MCP2517FD_REG_CiFIFOCON(2U), (uint8_t)(MCP2517FD_FIFOCON_TXEN | (3U << 5U)));
    /* Ensure TX FIFO is released from reset state (FRESET = 0) */
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(2U) + 1U), 0x00U);

    /* 11. Configure Filter 0 to accept ALL frames (STD & EXT) and store to FIFO 1 */
    mcp2517fd_write32(MCP2517FD_REG_CiFLTOBJ(0U), 0x00000000U); /* Filter ID 0 */
    mcp2517fd_write32(MCP2517FD_REG_CiMASK(0U), 0x00000000U);   /* Mask 0 = accept all, MIDE=0 */
    mcp2517fd_write8(MCP2517FD_REG_CiFLTCON(0U), 0x81U);        /* FLTEN=1, FBP=1 (FIFO 1) */

    /* 12. Enable Receive FIFO Interrupt in CiINT */
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiINT + 2U), (uint8_t)MCP2517FD_INT_RXIF);

    /* 13. Switch directly to Normal CAN FD Mode (Mode 0) */
    ret = mcp2517fd_request_mode(MCP2517FD_MODE_NORMAL_CANFD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter Normal CAN FD mode: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "MCP2517FD initialized in Normal CAN FD mode (Nominal: 500kbps 80%%, Data: 2Mbps 80%%, RX: 16 msgs, TX: 8 msgs)");
    }
    return ret;
}

esp_err_t mcp2517fd_self_test(void)
{
    ESP_LOGI(TAG, "Running internal loopback self-test...");
    esp_err_t err = mcp2517fd_request_mode(MCP2517FD_MODE_INT_LOOPBACK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Self-test: Failed to enter Internal Loopback mode: %s", esp_err_to_name(err));
        return err;
    }

    canfd_frame_t tx_test;
    (void)memset(&tx_test, 0, sizeof(tx_test));
    tx_test.id = 0x1A5U;
    tx_test.len = 8U;
    tx_test.data[0] = 0xAAU;
    tx_test.data[1] = 0x55U;
    tx_test.data[2] = 0x12U;
    tx_test.data[3] = 0x34U;
    tx_test.data[4] = 0x56U;
    tx_test.data[5] = 0x78U;
    tx_test.data[6] = 0x9AU;
    tx_test.data[7] = 0xBCU;
    tx_test.is_extended = false;
    tx_test.is_fd = true;
    tx_test.is_brs = true;

    err = mcp2517fd_transmit_frame(&tx_test);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Self-test: Loopback TX failed: %s", esp_err_to_name(err));
        (void)mcp2517fd_request_mode(MCP2517FD_MODE_NORMAL_CANFD);
        return err;
    }

    /* Wait up to 50ms for frame to loop back into RX FIFO */
    bool rx_ready = false;
    for (uint32_t i = 0U; i < 50U; i++) {
        if (mcp2517fd_has_rx_message()) {
            rx_ready = true;
            break;
        }
        esp_rom_delay_us(500);
    }

    if (!rx_ready) {
        ESP_LOGE(TAG, "Self-test: No message received in FIFO 1 within 50ms!");
        (void)mcp2517fd_request_mode(MCP2517FD_MODE_NORMAL_CANFD);
        return ESP_FAIL;
    }

    canfd_frame_t rx_test;
    (void)memset(&rx_test, 0, sizeof(rx_test));
    err = mcp2517fd_receive_frame(&rx_test);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Self-test: Failed to read looped back frame: %s", esp_err_to_name(err));
        (void)mcp2517fd_request_mode(MCP2517FD_MODE_NORMAL_CANFD);
        return err;
    }

    if ((rx_test.id != tx_test.id) || (rx_test.len != 8U) || (memcmp(rx_test.data, tx_test.data, 8U) != 0)) {
        ESP_LOGE(TAG, "Self-test: Data verification mismatch (got ID 0x%03lX, expected 0x1A5)",
                 (unsigned long)rx_test.id);
        (void)mcp2517fd_request_mode(MCP2517FD_MODE_NORMAL_CANFD);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "===============================================================");
    ESP_LOGI(TAG, ">>> SELF-TEST PASSED: Internal Loopback Successful!         <<<");
    ESP_LOGI(TAG, ">>> SPI, RAM, FIFOs, Filters & Registers are 100%% OK!       <<<");
    ESP_LOGI(TAG, "===============================================================");

    /* Reset FIFOs to clear self-test frames, then immediately re-enable them (FRESET = 0) */
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(1U) + 1U), (uint8_t)MCP2517FD_FIFOCON_FRESET);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(1U) + 1U), 0x00U);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(2U) + 1U), (uint8_t)MCP2517FD_FIFOCON_FRESET);
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(2U) + 1U), 0x00U);

    /* Return to Normal CAN FD mode */
    err = mcp2517fd_request_mode(MCP2517FD_MODE_NORMAL_CANFD);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter Normal CAN FD mode after self-test: %s", esp_err_to_name(err));
    }

    return err;
}

bool mcp2517fd_has_rx_message(void)
{
    bool has_msg = false;
    if ((s_spi_mutex != NULL) && (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(10U)) == pdTRUE)) {
        uint8_t status = mcp2517fd_read8(MCP2517FD_REG_CiFIFOSTA(1U));
        (void)xSemaphoreGive(s_spi_mutex);
        has_msg = ((status & MCP2517FD_FIFOSTA_TFNRFNIF) != 0U);
    }
    return has_msg;
}

esp_err_t mcp2517fd_receive_frame(canfd_frame_t *frame)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_spi_mutex != NULL) {
        if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    uint8_t fifo_sta = mcp2517fd_read8(MCP2517FD_REG_CiFIFOSTA(1U));
    if ((fifo_sta == 0xFFU) || ((fifo_sta & MCP2517FD_FIFOSTA_TFNRFNIF) == 0U)) {
        if (s_spi_mutex != NULL) {
            (void)xSemaphoreGive(s_spi_mutex);
        }
        return (fifo_sta == 0xFFU) ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_NOT_FOUND;
    }

    /* Check for overflow */
    if ((fifo_sta & MCP2517FD_FIFOSTA_RXOVIF) != 0U) {
        s_rx_overflow_count++;
        /* Clear overflow flag */
        mcp2517fd_write8(MCP2517FD_REG_CiFIFOSTA(1U), (uint8_t)(~MCP2517FD_FIFOSTA_RXOVIF));
    }

    /* Get User Address pointer for FIFO 1 */
    uint32_t ua = mcp2517fd_read32(MCP2517FD_REG_CiFIFOUA(1U));
    uint16_t ram_addr = (uint16_t)(MCP2517FD_RAM_ADDR_START + (ua & 0x0FFFU));

    /* Burst read entire message object: 8 bytes header + up to 64 bytes data = 72 bytes */
    uint8_t msg_buf[72] = {0U};
    esp_err_t err = mcp2517fd_read_regs(ram_addr, msg_buf, sizeof(msg_buf));
    if (err != ESP_OK) {
        if (s_spi_mutex != NULL) {
            (void)xSemaphoreGive(s_spi_mutex);
        }
        return err;
    }

    /* Increment FIFO pointer (UINC = bit 0 of FIFOCON1 byte 1 at 0x05D) */
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(1U) + 1U), (uint8_t)MCP2517FD_FIFOCON_UINC);

    /* Release SPI mutex as soon as SPI transfers are done */
    if (s_spi_mutex != NULL) {
        (void)xSemaphoreGive(s_spi_mutex);
    }

    /* Decode Header */
    uint32_t raw_id = (uint32_t)msg_buf[0] |
                      ((uint32_t)msg_buf[1] << 8U) |
                      ((uint32_t)msg_buf[2] << 16U) |
                      ((uint32_t)msg_buf[3] << 24U);

    uint32_t flags = (uint32_t)msg_buf[4] |
                     ((uint32_t)msg_buf[5] << 8U) |
                     ((uint32_t)msg_buf[6] << 16U) |
                     ((uint32_t)msg_buf[7] << 24U);

    uint8_t dlc = (uint8_t)(flags & 0x0FU);
    frame->len = s_dlc_to_len[dlc];
    frame->is_extended = (flags & (1UL << 4U)) != 0U;
    frame->is_rtr = (flags & (1UL << 5U)) != 0U;
    frame->is_brs = (flags & (1UL << 6U)) != 0U;
    frame->is_fd  = (flags & (1UL << 7U)) != 0U;
    frame->filter_idx = (uint8_t)((flags >> 11U) & 0x1FU);

    if (frame->is_extended) {
        /* Extended ID: bits [10:0] = SID, bits [28:11] = EID */
        frame->id = ((raw_id >> 11U) & 0x3FFFFU) | ((raw_id & 0x7FFU) << 18U);
    } else {
        frame->id = raw_id & 0x7FFU;
    }

    /* Copy payload */
    if (frame->len > 64U) {
        frame->len = 64U;
    }
    (void)memcpy(frame->data, &msg_buf[8], frame->len);

    return ESP_OK;
}

esp_err_t mcp2517fd_transmit_frame(const canfd_frame_t *frame)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_spi_mutex != NULL) {
        if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(100U)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    /* Check if TX FIFO 2 has space (TFNRFNIF = 1 means not full) */
    uint8_t fifo_sta = mcp2517fd_read8(MCP2517FD_REG_CiFIFOSTA(2U));

    /* If FIFO 2 suffered an error or abort on a busy bus, reset FIFO 2 to restore transmission */
    if ((fifo_sta & (MCP2517FD_FIFOSTA_TXABT | MCP2517FD_FIFOSTA_TXERR)) != 0U) {
        mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(2U) + 1U), (uint8_t)MCP2517FD_FIFOCON_FRESET);
        mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(2U) + 1U), 0x00U);
        fifo_sta = mcp2517fd_read8(MCP2517FD_REG_CiFIFOSTA(2U));
    }

    if ((fifo_sta & MCP2517FD_FIFOSTA_TFNRFNIF) == 0U) {
        if (s_spi_mutex != NULL) {
            (void)xSemaphoreGive(s_spi_mutex);
        }
        return ESP_ERR_TIMEOUT; /* TX FIFO full (waiting for previous transmission) */
    }

    /* Get User Address pointer for FIFO 2 */
    uint32_t ua = mcp2517fd_read32(MCP2517FD_REG_CiFIFOUA(2U));
    uint16_t ram_addr = (uint16_t)(MCP2517FD_RAM_ADDR_START + (ua & 0x0FFFU));

    /* Prepare Header */
    uint32_t idf = frame->id;
    if (frame->is_extended) {
        idf = ((frame->id >> 18U) & 0x7FFU) | ((frame->id & 0x3FFFFU) << 11U);
    }

    uint32_t flags = (uint32_t)len_to_dlc(frame->len);
    if (frame->is_extended) {
        flags |= (1UL << 4U);
    }
    if (frame->is_rtr) {
        flags |= (1UL << 5U);
    }
    if (frame->is_brs) {
        flags |= (1UL << 6U);
    }
    if (frame->is_fd) {
        flags |= (1UL << 7U);
    }

    uint8_t tx_buf[72] = {0U};
    tx_buf[0] = (uint8_t)(idf & 0xFFU);
    tx_buf[1] = (uint8_t)((idf >> 8U) & 0xFFU);
    tx_buf[2] = (uint8_t)((idf >> 16U) & 0xFFU);
    tx_buf[3] = (uint8_t)((idf >> 24U) & 0xFFU);

    tx_buf[4] = (uint8_t)(flags & 0xFFU);
    tx_buf[5] = (uint8_t)((flags >> 8U) & 0xFFU);
    tx_buf[6] = (uint8_t)((flags >> 16U) & 0xFFU);
    tx_buf[7] = (uint8_t)((flags >> 24U) & 0xFFU);

    uint8_t copy_len = (frame->len > 64U) ? 64U : frame->len;
    (void)memcpy(&tx_buf[8], frame->data, copy_len);

    size_t write_len = 8U + (((size_t)copy_len + 3U) / 4U) * 4U;
    esp_err_t err = mcp2517fd_write_regs(ram_addr, tx_buf, write_len);
    if (err != ESP_OK) {
        if (s_spi_mutex != NULL) {
            (void)xSemaphoreGive(s_spi_mutex);
        }
        return err;
    }

    /* Increment FIFO pointer, set unlimited retransmissions, and request transmission */
    mcp2517fd_write8((uint16_t)(MCP2517FD_REG_CiFIFOCON(2U) + 1U),
                     (uint8_t)(MCP2517FD_FIFOCON_TXAT_UNLIMITED | MCP2517FD_FIFOCON_UINC | MCP2517FD_FIFOCON_TXREQ));

    if (s_spi_mutex != NULL) {
        (void)xSemaphoreGive(s_spi_mutex);
    }
    return ESP_OK;
}

uint32_t mcp2517fd_get_rx_overflow_count(void)
{
    return s_rx_overflow_count;
}

esp_err_t mcp2517fd_get_diag(mcp2517fd_diag_t *diag)
{
    if (diag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_spi_mutex != NULL) {
        if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    uint32_t trec = mcp2517fd_read32(MCP2517FD_REG_CiTREC);
    uint32_t bdiag0 = mcp2517fd_read32(MCP2517FD_REG_CiBDIAG0);
    uint32_t bdiag1 = mcp2517fd_read32(MCP2517FD_REG_CiBDIAG1);

    diag->trec_raw = trec;
    diag->bdiag0_raw = bdiag0;
    diag->bdiag1_raw = bdiag1;

    diag->rec = (uint8_t)(trec & 0xFFU);
    diag->tec = (uint8_t)((trec >> 8U) & 0xFFU);

    diag->txbo   = (trec & MCP2517FD_TREC_TXBO) != 0U;
    diag->txbp   = (trec & MCP2517FD_TREC_TXBP) != 0U;
    diag->rxbp   = (trec & MCP2517FD_TREC_RXBP) != 0U;
    diag->txwarn = (trec & MCP2517FD_TREC_TXWARN) != 0U;
    diag->rxwarn = (trec & MCP2517FD_TREC_RXWARN) != 0U;

    diag->nbit0_err  = (bdiag1 & MCP2517FD_BDIAG1_NBIT0_ERR) != 0U;
    diag->nbit1_err  = (bdiag1 & MCP2517FD_BDIAG1_NBIT1_ERR) != 0U;
    diag->nack_err   = (bdiag1 & MCP2517FD_BDIAG1_NACK_ERR) != 0U;
    diag->nstuff_err = (bdiag1 & MCP2517FD_BDIAG1_NSTUFF_ERR) != 0U;
    diag->ncrc_err   = (bdiag1 & MCP2517FD_BDIAG1_NCRC_ERR) != 0U;
    diag->dbit0_err  = (bdiag1 & MCP2517FD_BDIAG1_DBIT0_ERR) != 0U;
    diag->dbit1_err  = (bdiag1 & MCP2517FD_BDIAG1_DBIT1_ERR) != 0U;

    /* Clear BDIAG1 latched error bits */
    if (bdiag1 != 0U) {
        mcp2517fd_write32(MCP2517FD_REG_CiBDIAG1, 0x00000000U);
    }

    if (s_spi_mutex != NULL) {
        (void)xSemaphoreGive(s_spi_mutex);
    }
    return ESP_OK;
}

static esp_err_t mcp2517fd_calculate_timing(uint32_t nominal_bps, uint32_t data_bps, mcp2517fd_osc_t osc,
                                            uint32_t *out_nbtcfg, uint32_t *out_dbtcfg, uint32_t *out_tdc)
{
    if ((out_nbtcfg == NULL) || (out_dbtcfg == NULL) || (out_tdc == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (osc == MCP2517FD_OSC_40MHZ) {
        /* 40 MHz Oscillator */
        switch (nominal_bps) {
        case 1000000U:
            *out_nbtcfg = (0UL << 24U) | (30UL << 16U) | (7UL << 8U) | 7UL; /* 40 TQ, 80% */
            break;
        case 500000U:
            *out_nbtcfg = (4UL << 24U) | (11UL << 16U) | (2UL << 8U) | 2UL; /* 16 TQ, 81.25% (PCAN) */
            break;
        case 250000U:
            *out_nbtcfg = (9UL << 24U) | (11UL << 16U) | (2UL << 8U) | 2UL; /* 16 TQ, 81.25% */
            break;
        case 125000U:
            *out_nbtcfg = (19UL << 24U) | (11UL << 16U) | (2UL << 8U) | 2UL; /* 16 TQ, 81.25% */
            break;
        case 100000U:
            *out_nbtcfg = (19UL << 24U) | (14UL << 16U) | (3UL << 8U) | 3UL; /* 20 TQ, 80% */
            break;
        case 50000U:
            *out_nbtcfg = (39UL << 24U) | (14UL << 16U) | (3UL << 8U) | 3UL; /* 20 TQ, 80% */
            break;
        default:
            ESP_LOGE(TAG, "Unsupported nominal bitrate: %lu bps", (unsigned long)nominal_bps);
            return ESP_ERR_NOT_SUPPORTED;
        }

        switch (data_bps) {
        case 5000000U:
            *out_dbtcfg = (0UL << 24U) | (4UL << 16U) | (1UL << 8U) | 1UL;
            *out_tdc = (1UL << 25U) | (1UL << 17U) | (5UL << 8U);
            break;
        case 4000000U:
            *out_dbtcfg = (0UL << 24U) | (6UL << 16U) | (1UL << 8U) | 1UL;
            *out_tdc = (1UL << 25U) | (1UL << 17U) | (7UL << 8U);
            break;
        case 2000000U:
            *out_dbtcfg = (0UL << 24U) | (14UL << 16U) | (3UL << 8U) | 3UL;
            *out_tdc = (1UL << 25U) | (1UL << 17U) | (15UL << 8U);
            break;
        case 1000000U:
        case 0U: /* 0 means match nominal or Classic CAN */
            *out_dbtcfg = (0UL << 24U) | (30UL << 16U) | (7UL << 8U) | 7UL;
            *out_tdc = (1UL << 25U) | (1UL << 17U) | (31UL << 8U);
            break;
        default:
            ESP_LOGE(TAG, "Unsupported data bitrate: %lu bps", (unsigned long)data_bps);
            return ESP_ERR_NOT_SUPPORTED;
        }
    } else {
        /* 20 MHz Oscillator */
        switch (nominal_bps) {
        case 1000000U:
            *out_nbtcfg = (0UL << 24U) | (14UL << 16U) | (3UL << 8U) | 3UL;
            break;
        case 500000U:
            *out_nbtcfg = (0UL << 24U) | (30UL << 16U) | (7UL << 8U) | 7UL;
            break;
        case 250000U:
            *out_nbtcfg = (1UL << 24U) | (30UL << 16U) | (7UL << 8U) | 7UL;
            break;
        case 125000U:
            *out_nbtcfg = (3UL << 24U) | (30UL << 16U) | (7UL << 8U) | 7UL;
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
        }

        switch (data_bps) {
        case 2000000U:
            *out_dbtcfg = (0UL << 24U) | (6UL << 16U) | (1UL << 8U) | 1UL;
            *out_tdc = (1UL << 25U) | (1UL << 17U) | (7UL << 8U);
            break;
        case 1000000U:
        case 0U:
            *out_dbtcfg = (0UL << 24U) | (14UL << 16U) | (3UL << 8U) | 3UL;
            *out_tdc = (1UL << 25U) | (1UL << 17U) | (15UL << 8U);
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
        }
    }

    return ESP_OK;
}

esp_err_t mcp2517fd_set_bitrate(uint32_t nominal_bps, uint32_t data_bps)
{
    uint32_t nbtcfg = 0U;
    uint32_t dbtcfg = 0U;
    uint32_t tdc = 0U;

    esp_err_t err = mcp2517fd_calculate_timing(nominal_bps, data_bps, s_current_osc, &nbtcfg, &dbtcfg, &tdc);
    if (err != ESP_OK) {
        return err;
    }

    if (s_spi_mutex != NULL) {
        if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(100U)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    uint8_t prev_mode = (uint8_t)((mcp2517fd_read8((uint16_t)(MCP2517FD_REG_CiCON + 2U)) >> 5U) & 0x07U);

    err = mcp2517fd_request_mode(MCP2517FD_MODE_CONFIGURATION);
    if (err == ESP_OK) {
        mcp2517fd_write32(MCP2517FD_REG_CiNBTCFG, nbtcfg);
        mcp2517fd_write32(MCP2517FD_REG_CiDBTCFG, dbtcfg);
        mcp2517fd_write32(MCP2517FD_REG_CiTDC, tdc);
        s_current_nominal_bps = nominal_bps;
        s_current_data_bps = data_bps;

        uint8_t target_mode = (prev_mode == MCP2517FD_MODE_CONFIGURATION) ? MCP2517FD_MODE_NORMAL_CANFD : prev_mode;
        err = mcp2517fd_request_mode(target_mode);
        s_current_mode = target_mode;
        ESP_LOGI(TAG, "Bitrate updated: Nominal=%lu bps, Data=%lu bps",
                 (unsigned long)nominal_bps, (unsigned long)data_bps);
    }

    if (s_spi_mutex != NULL) {
        (void)xSemaphoreGive(s_spi_mutex);
    }
    return err;
}

esp_err_t mcp2517fd_set_mode(uint8_t mode)
{
    if (s_spi_mutex != NULL) {
        if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(100U)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    esp_err_t err = mcp2517fd_request_mode(mode);
    if (err == ESP_OK) {
        s_current_mode = mode;
        ESP_LOGI(TAG, "Mode updated to %u", (unsigned int)mode);
    }

    if (s_spi_mutex != NULL) {
        (void)xSemaphoreGive(s_spi_mutex);
    }
    return err;
}

esp_err_t mcp2517fd_set_filter(uint8_t filter_idx, uint32_t filter_id, uint32_t mask, bool is_ext)
{
    if (filter_idx >= 32U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_spi_mutex != NULL) {
        if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(100U)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    uint8_t prev_mode = (uint8_t)((mcp2517fd_read8((uint16_t)(MCP2517FD_REG_CiCON + 2U)) >> 5U) & 0x07U);
    esp_err_t err = mcp2517fd_request_mode(MCP2517FD_MODE_CONFIGURATION);
    if (err == ESP_OK) {
        uint32_t fltobj;
        uint32_t maskreg;

        if (mask == 0U) {
            fltobj = 0U;
            maskreg = 0U; /* MIDE=0: match both STD and EXT */
        } else if (is_ext) {
            fltobj = (filter_id & 0x3FFFFU) | (((filter_id >> 18U) & 0x7FFU) << 18U) | (1UL << 30U);
            maskreg = (mask & 0x3FFFFU) | (((mask >> 18U) & 0x7FFU) << 18U) | (1UL << 30U);
        } else {
            fltobj = filter_id & 0x7FFU;
            maskreg = (mask & 0x7FFU) | (1UL << 30U);
        }

        mcp2517fd_write32(MCP2517FD_REG_CiFLTOBJ(filter_idx), fltobj);
        mcp2517fd_write32(MCP2517FD_REG_CiMASK(filter_idx), maskreg);
        mcp2517fd_write8(MCP2517FD_REG_CiFLTCON(filter_idx), 0x81U);

        uint8_t target_mode = (prev_mode == MCP2517FD_MODE_CONFIGURATION) ? MCP2517FD_MODE_NORMAL_CANFD : prev_mode;
        err = mcp2517fd_request_mode(target_mode);
        s_current_mode = target_mode;
        ESP_LOGI(TAG, "Filter %u updated: ID=0x%08lX, Mask=0x%08lX, Ext=%d",
                 (unsigned int)filter_idx, (unsigned long)filter_id, (unsigned long)mask, (int)is_ext);
    }

    if (s_spi_mutex != NULL) {
        (void)xSemaphoreGive(s_spi_mutex);
    }
    return err;
}

esp_err_t mcp2517fd_bus_control(bool enable)
{
    if (enable) {
        (void)gpio_set_level(MCP2517FD_PIN_STBY0, 0);
        (void)gpio_set_level(MCP2517FD_PIN_STBY1, 0);
        return mcp2517fd_set_mode(MCP2517FD_MODE_NORMAL_CANFD);
    } else {
        esp_err_t err = mcp2517fd_set_mode(MCP2517FD_MODE_CONFIGURATION);
        (void)gpio_set_level(MCP2517FD_PIN_STBY0, 1);
        (void)gpio_set_level(MCP2517FD_PIN_STBY1, 1);
        return err;
    }
}

uint8_t mcp2517fd_get_mode(void)
{
    return s_current_mode;
}

void mcp2517fd_get_bitrates(uint32_t *nominal_bps, uint32_t *data_bps)
{
    if (nominal_bps != NULL) {
        *nominal_bps = s_current_nominal_bps;
    }
    if (data_bps != NULL) {
        *data_bps = s_current_data_bps;
    }
}
