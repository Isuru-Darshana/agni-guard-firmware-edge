#include "sx1278.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SX1278";

// ── SPI primitives ──────────────────────────────────────────

uint8_t sx1278_read_reg(sx1278_t *dev, uint8_t addr) {
    uint8_t tx[2] = { addr & 0x7F, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(dev->spi, &t);
    return rx[1];
}

static void sx1278_write_reg(sx1278_t *dev, uint8_t addr, uint8_t val) {
    uint8_t tx[2] = { addr | 0x80, val };
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
    };
    spi_device_transmit(dev->spi, &t);
}

static void sx1278_write_fifo(sx1278_t *dev,
                               const uint8_t *data, uint8_t len) {
    uint8_t tx[257];
    tx[0] = REG_FIFO | 0x80;
    memcpy(&tx[1], data, len);
    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
    };
    spi_device_transmit(dev->spi, &t);
}

static void sx1278_read_fifo(sx1278_t *dev,
                              uint8_t *data, uint8_t len) {
    uint8_t tx[257] = { REG_FIFO & 0x7F };
    uint8_t rx[257] = { 0 };
    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(dev->spi, &t);
    memcpy(data, &rx[1], len);
}

// ── Frequency: 433MHz ───────────────────────────────────────
// Frf = 433000000 / (32000000 / 2^19) = 7094272 = 0x6C8000
static void sx1278_set_frequency(sx1278_t *dev, uint32_t freq_hz) {
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ULL;
    sx1278_write_reg(dev, REG_FRF_MSB, (frf >> 16) & 0xFF);
    sx1278_write_reg(dev, REG_FRF_MID, (frf >>  8) & 0xFF);
    sx1278_write_reg(dev, REG_FRF_LSB, (frf >>  0) & 0xFF);
}

// ── Init ────────────────────────────────────────────────────
esp_err_t sx1278_init(sx1278_t *dev,
                      spi_host_device_t host,
                      gpio_num_t mosi, gpio_num_t miso,
                      gpio_num_t sck,  gpio_num_t nss,
                      gpio_num_t rst,  gpio_num_t dio0) {
    dev->pin_rst  = rst;
    dev->pin_dio0 = dio0;

    // SPI bus
    spi_bus_config_t bus = {
        .mosi_io_num   = mosi,
        .miso_io_num   = miso,
        .sclk_io_num   = sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO));

    // SX1278: SPI mode 0, max 10MHz
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8000000,
        .mode           = 0,
        .spics_io_num   = nss,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(host, &dev_cfg, &dev->spi));

    // GPIO
    gpio_set_direction(rst,  GPIO_MODE_OUTPUT);
    gpio_set_direction(dio0, GPIO_MODE_INPUT);

    // Hardware reset — hold low 1ms, release, wait 10ms
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(rst, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify chip version — SX1278 returns 0x12
    uint8_t ver = sx1278_read_reg(dev, REG_VERSION);
    if (ver != 0x12) {
        ESP_LOGE(TAG, "SX1278 not found! version=0x%02X (expected 0x12)", ver);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SX1278 found version=0x%02X", ver);

    // CRITICAL: Must be in sleep mode to switch to LoRa mode
    sx1278_write_reg(dev, REG_OP_MODE, MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable LoRa mode (bit7=1) + sleep
    sx1278_write_reg(dev, REG_OP_MODE,
                     MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Confirm LoRa mode set
    uint8_t mode = sx1278_read_reg(dev, REG_OP_MODE);
    if (!(mode & MODE_LONG_RANGE_MODE)) {
        ESP_LOGE(TAG, "Failed to enter LoRa mode! RegOpMode=0x%02X", mode);
        return ESP_FAIL;
    }

    // 433MHz frequency
    sx1278_set_frequency(dev, 433000000);

    // FIFO base addresses
    sx1278_write_reg(dev, REG_FIFO_TX_BASE_ADDR, 0x00);
    sx1278_write_reg(dev, REG_FIFO_RX_BASE_ADDR, 0x00);

    // LNA: max gain, LNA boost LF (for 433MHz LF port)
    sx1278_write_reg(dev, REG_LNA, 0x23);

    // ModemConfig1: BW=125kHz, CR=4/5, ExplicitHeader
    // Bits[7:4]=0111(125kHz) Bits[3:1]=001(CR4/5) Bit[0]=0(explicit)
    sx1278_write_reg(dev, REG_MODEM_CONFIG_1, 0x72);

    // ModemConfig2: SF=10, normal TX, CRC on
    // Bits[7:4]=1010(SF10) Bit[3]=0(normal) Bit[2]=1(CRC)
    sx1278_write_reg(dev, REG_MODEM_CONFIG_2, 0xA4);

    // ModemConfig3: AGC on, LowDataRateOptimize off
    // SF10 @ 125kHz → symbol time = 8.19ms < 16ms → LDRO not needed
    // Bit[2]=1(AGC) Bit[3]=0(LDRO off)
    sx1278_write_reg(dev, REG_MODEM_CONFIG_3, 0x04);

    // TX power: PA_BOOST pin, +17dBm
    // PA_BOOST = bit7, MaxPower = 0b111, OutputPower = 0b1111
    // Pout = 17 - (15 - OutputPower) = 17dBm
    sx1278_write_reg(dev, REG_PA_CONFIG, 0x8F);

    // PA DAC: standard power (not +20dBm boost)
    sx1278_write_reg(dev, REG_PA_DAC, 0x84);

    // OCP: 100mA overcurrent protection
    sx1278_write_reg(dev, REG_OCP, 0x2B);

    // Preamble: 8 symbols
    sx1278_write_reg(dev, REG_PREAMBLE_MSB, 0x00);
    sx1278_write_reg(dev, REG_PREAMBLE_LSB, 0x08);

    // Sync word: 0x12 (private network, not LoRaWAN 0x34)
    sx1278_write_reg(dev, REG_SYNC_WORD, 0x12);

    // SF6 detection — not SF6 but required for correct operation
    sx1278_write_reg(dev, REG_DETECT_OPTIMIZE,  0xC3);
    sx1278_write_reg(dev, REG_DETECTION_THRESH, 0x0A);

    // Go to standby
    sx1278_write_reg(dev, REG_OP_MODE,
                     MODE_LONG_RANGE_MODE | MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Init OK: 433MHz SF10 BW125kHz CR4/5 CRC+ +17dBm");
    return ESP_OK;
}

// ── Standby ─────────────────────────────────────────────────
void sx1278_standby(sx1278_t *dev) {
    sx1278_write_reg(dev, REG_OP_MODE,
                     MODE_LONG_RANGE_MODE | MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// ── Sleep ───────────────────────────────────────────────────
void sx1278_sleep(sx1278_t *dev) {
    sx1278_write_reg(dev, REG_OP_MODE,
                     MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

// ── Transmit ─────────────────────────────────────────────────
esp_err_t sx1278_send(sx1278_t *dev,
                      const uint8_t *data, uint8_t len) {
    // Go to standby before loading FIFO
    sx1278_standby(dev);

    // Reset FIFO pointer to TX base
    sx1278_write_reg(dev, REG_FIFO_ADDR_PTR,   0x00);
    sx1278_write_reg(dev, REG_PAYLOAD_LENGTH,  len);

    // Write payload to FIFO
    sx1278_write_fifo(dev, data, len);

    // DIO0 → TxDone (mapping 01 for LoRa TxDone)
    sx1278_write_reg(dev, REG_DIO_MAPPING_1, 0x40);

    // Clear all IRQ flags
    sx1278_write_reg(dev, REG_IRQ_FLAGS, 0xFF);

    // Enter TX mode
    sx1278_write_reg(dev, REG_OP_MODE,
                     MODE_LONG_RANGE_MODE | MODE_TX);

    // Wait for TxDone on DIO0 (max 5s)
    uint32_t start = xTaskGetTickCount();
    while (!gpio_get_level(dev->pin_dio0)) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(5000)) {
            ESP_LOGE(TAG, "TX timeout");
            sx1278_standby(dev);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Clear TxDone IRQ
    sx1278_write_reg(dev, REG_IRQ_FLAGS, IRQ_TX_DONE);

    // Back to standby
    sx1278_standby(dev);

    ESP_LOGD(TAG, "TX done (%d bytes)", len);
    return ESP_OK;
}

// ── Receive (blocking with timeout) ─────────────────────────
esp_err_t sx1278_receive(sx1278_t *dev,
                         uint8_t *buf, uint8_t *len,
                         int8_t *rssi,
                         uint32_t timeout_ms) {
    // Reset FIFO RX pointer
    sx1278_write_reg(dev, REG_FIFO_ADDR_PTR,     0x00);
    sx1278_write_reg(dev, REG_FIFO_RX_BASE_ADDR, 0x00);

    // DIO0 → RxDone (mapping 00 for LoRa RxDone)
    sx1278_write_reg(dev, REG_DIO_MAPPING_1, 0x00);

    // Clear all IRQ flags
    sx1278_write_reg(dev, REG_IRQ_FLAGS, 0xFF);

    // Enter RX continuous mode
    sx1278_write_reg(dev, REG_OP_MODE,
                     MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    // Wait for RxDone on DIO0
    uint32_t start = xTaskGetTickCount();
    while (!gpio_get_level(dev->pin_dio0)) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            sx1278_standby(dev);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Read IRQ flags
    uint8_t irq = sx1278_read_reg(dev, REG_IRQ_FLAGS);

    // Check CRC error
    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        sx1278_write_reg(dev, REG_IRQ_FLAGS, 0xFF);
        ESP_LOGW(TAG, "CRC error");
        sx1278_standby(dev);
        return ESP_ERR_INVALID_CRC;
    }

    // Read packet from FIFO
    uint8_t rx_addr = sx1278_read_reg(dev, REG_FIFO_RX_CURRENT);
    uint8_t rx_len  = sx1278_read_reg(dev, REG_RX_NB_BYTES);

    sx1278_write_reg(dev, REG_FIFO_ADDR_PTR, rx_addr);
    sx1278_read_fifo(dev, buf, rx_len);
    *len = rx_len;

    // RSSI formula for LF port (433MHz < 525MHz):
    // RSSI[dBm] = -164 + RegPktRssiValue
    // NOT -157 which is for HF port (868/915MHz)
    *rssi = (int8_t)(-164 + (int16_t)sx1278_read_reg(dev,
                                                       REG_PKT_RSSI_VALUE));

    // Clear all IRQ flags
    sx1278_write_reg(dev, REG_IRQ_FLAGS, 0xFF);

    // Back to standby
    sx1278_standby(dev);

    ESP_LOGD(TAG, "RX done: %d bytes RSSI=%ddBm", rx_len, *rssi);
    return ESP_OK;
}