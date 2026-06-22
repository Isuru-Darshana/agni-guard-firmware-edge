#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include <stdint.h>
#include <stdbool.h>

// ── BME680 I2C config ─────────────────────────────────────────
// Shares I2C_NUM_0 with BME280 (BME280=0x76, BME680=0x77)
#define BME680_I2C_ADDR   0x77   // SDO pulled HIGH on your PCB
#define BME680_I2C_PORT   I2C_NUM_0
#define BME680_SDA_PIN    GPIO_NUM_8
#define BME680_SCL_PIN    GPIO_NUM_9

// ── BSEC sample rate ─────────────────────────────────────────
// LP = 3 second interval (low power, matches 33v_3s_4d config)
#define BSEC_SAMPLE_RATE  BSEC_SAMPLE_RATE_LP

// ── Output data struct ────────────────────────────────────────
typedef struct {
    float    temperature;      // °C (compensated)
    float    humidity;         // %RH (compensated)
    float    pressure;         // hPa
    float    gas_resistance;   // kΩ raw gas resistance
    float    iaq;              // 0-500 IAQ index
    float    co2_equivalent;   // ppm CO2 equivalent
    float    voc_equivalent;   // ppm VOC equivalent
    uint8_t  iaq_accuracy;     // 0=unreliable 1=low 2=medium 3=high
    bool     gas_valid;        // true = gas measurement valid
    int64_t  timestamp_ns;     // nanoseconds since boot
} bme680_data_t;

// ── API ───────────────────────────────────────────────────────
esp_err_t bme680_bsec_init(void);
esp_err_t bme680_bsec_read(bme680_data_t *data);
esp_err_t bme680_bsec_save_state(void);
esp_err_t bme680_bsec_load_state(void);
void      bme680_bsec_get_version(uint8_t *major, uint8_t *minor,
                                   uint8_t *major_bugfix,
                                   uint8_t *minor_bugfix);