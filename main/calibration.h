#pragma once

#include "esp_err.h"
#include "bme280.h"
#include "pms7003.h"
#include <stdbool.h>
#include <stdint.h>

#define CALIBRATION_DURATION_MS  (60UL * 60UL * 1000UL)
#define CALIBRATION_INTERVAL_MS  10000
#define CALIBRATION_PROMPT_MS    30000

typedef struct {
    float    gas_baseline;
    uint16_t pm25_baseline;
    uint32_t readings;
} calibration_result_t;

// Returns true if user wants to calibrate, false to skip
bool      calibration_prompt(void);

// Runs 60-min calibration, fills result
esp_err_t calibration_run(calibration_result_t *result);