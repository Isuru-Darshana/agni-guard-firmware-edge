#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "bme280.h"
#include "pms7003.h"
#include "sleep_manager.h"
#include <stdint.h>

// SD SPI pins — your PCB
#define SD_CS    GPIO_NUM_38
#define SD_MOSI  GPIO_NUM_35
#define SD_SCK   GPIO_NUM_36
#define SD_MISO  GPIO_NUM_37

typedef struct {
    uint32_t       boot;
    uint16_t       seq;
    uint8_t        node_id;

    // BME680 (placeholder until BSEC)
    float          bme680_temp;
    float          bme680_humidity;
    float          bme680_pressure;
    float          bme680_gas;

    // BME280
    bme280_data_t  bme280;

    // PMS7003
    pms7003_data_t pms;

    // Battery — DS2782
    uint8_t        soc;
    float          battery_voltage;  // pack total V

    // Stage
    fire_stage_t   stage;
} sd_log_entry_t;

esp_err_t sd_logger_init(void);
esp_err_t sd_logger_write(const sd_log_entry_t *entry,
                           uint8_t node_id);
void      sd_logger_deinit(void);