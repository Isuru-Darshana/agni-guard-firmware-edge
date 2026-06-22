#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#define BME280_I2C_ADDR    0x76
#define BME280_I2C_PORT    I2C_NUM_0
#define BME280_SDA_PIN     GPIO_NUM_8
#define BME280_SCL_PIN     GPIO_NUM_9
#define BME280_I2C_FREQ    100000

typedef struct {
    float temperature;   // °C
    float humidity;      // %RH
    float pressure;      // hPa
} bme280_data_t;

esp_err_t bme280_init(void);
esp_err_t bme280_read(bme280_data_t *data);