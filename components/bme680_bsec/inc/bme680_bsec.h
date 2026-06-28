#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ── BME680 I2C config ─────────────────────────────────────────
#define BME680_I2C_ADDR   0x77
#define BME680_I2C_PORT   I2C_NUM_1
#define BME680_SDA_PIN    GPIO_NUM_41
#define BME680_SCL_PIN    GPIO_NUM_42

// ── Output data struct ────────────────────────────────────────
typedef struct {
    float    temperature;      // °C
    float    humidity;         // %RH
    float    pressure;         // hPa
    float    gas_resistance;   // kΩ raw gas resistance
    bool     gas_valid;        // true = gas measurement stable (heater warm)
    int64_t  timestamp_ns;
} bme680_data_t;

// ── API ───────────────────────────────────────────────────────
esp_err_t bme680_bsec_init(void);
esp_err_t bme680_bsec_read(bme680_data_t *data);
