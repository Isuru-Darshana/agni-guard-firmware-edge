#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ── Your PCB pin definitions ────────────────────────────────
// Edge node (HSPI)
#define SX1278_EDGE_NSS   GPIO_NUM_10
#define SX1278_EDGE_MOSI  GPIO_NUM_11
#define SX1278_EDGE_SCK   GPIO_NUM_12
#define SX1278_EDGE_MISO  GPIO_NUM_13
#define SX1278_EDGE_RST   GPIO_NUM_7
#define SX1278_EDGE_DIO0  GPIO_NUM_15
#define SX1278_EDGE_SPI   SPI2_HOST

// ── SX1278 Register Map ─────────────────────────────────────
// Common
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_OCP                 0x0B
#define REG_LNA                 0x0C

// LoRa specific
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_FIFO_RX_CURRENT     0x10
#define REG_IRQ_FLAGS_MASK      0x11
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_PKT_SNR_VALUE       0x19
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_PREAMBLE_MSB        0x20
#define REG_PREAMBLE_LSB        0x21
#define REG_PAYLOAD_LENGTH      0x22
#define REG_MODEM_CONFIG_3      0x26
#define REG_DETECT_OPTIMIZE     0x31
#define REG_DETECTION_THRESH    0x37
#define REG_SYNC_WORD           0x39
#define REG_DIO_MAPPING_1       0x40
#define REG_VERSION             0x42
#define REG_PA_DAC              0x4D

// ── OpMode values ───────────────────────────────────────────
#define MODE_LONG_RANGE_MODE    0x80
#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_TX                 0x03
#define MODE_RX_CONTINUOUS      0x05

// ── IRQ flags (RegIrqFlags 0x12) ───────────────────────────
#define IRQ_RX_DONE             0x40
#define IRQ_TX_DONE             0x08
#define IRQ_PAYLOAD_CRC_ERROR   0x20

// ── Device handle ───────────────────────────────────────────
typedef struct {
    spi_device_handle_t spi;
    gpio_num_t          pin_rst;
    gpio_num_t          pin_dio0;
} sx1278_t;

// ── API ─────────────────────────────────────────────────────
esp_err_t sx1278_init(sx1278_t *dev,
                      spi_host_device_t host,
                      gpio_num_t mosi, gpio_num_t miso,
                      gpio_num_t sck,  gpio_num_t nss,
                      gpio_num_t rst,  gpio_num_t dio0);

esp_err_t sx1278_send(sx1278_t *dev,
                      const uint8_t *data, uint8_t len);

esp_err_t sx1278_receive(sx1278_t *dev,
                         uint8_t *buf, uint8_t *len,
                         int8_t *rssi,
                         uint32_t timeout_ms);

void      sx1278_sleep(sx1278_t *dev);
void      sx1278_standby(sx1278_t *dev);
uint8_t   sx1278_read_reg(sx1278_t *dev, uint8_t addr);