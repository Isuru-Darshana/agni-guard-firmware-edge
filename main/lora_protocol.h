#pragma once

#include "esp_err.h"
#include "sleep_manager.h"
#include "sx1278.h"
#include <stdint.h>

#define PKT_TYPE_DATA  'D'
#define PKT_TYPE_ACK   'A'
#define ACK_TIMEOUT_MS 30000

typedef struct {
    // BME680 — heat compensated + raw gas
    float    bme680_temp;
    float    bme680_humidity;
    float    bme680_pressure;
    float    bme680_gas;       // kΩ raw gas resistance

    // BME280
    float    bme280_temp;
    float    bme280_humidity;
    float    bme280_pressure;

    // PMS7003
    uint16_t pm2_5;
    uint16_t pm10;

    // Battery
    uint8_t  battery_soc;
    float    battery_voltage;

    // Calibration
    float    gas_baseline;
    uint16_t sequence;
} lora_packet_t;

esp_err_t lora_transmit(sx1278_t *dev,
                        const lora_packet_t *pkt,
                        uint8_t node_id);

esp_err_t lora_wait_ack(sx1278_t *dev,
                        uint8_t node_id,
                        fire_stage_t *stage_out);