#pragma once

#include "esp_err.h"
#include <stdbool.h>

esp_err_t nvs_config_init(void);
esp_err_t nvs_config_save_baseline(float gas_baseline);
esp_err_t nvs_config_load_baseline(float *gas_baseline);
esp_err_t nvs_config_save_calibration_flag(bool done);
esp_err_t nvs_config_load_calibration_flag(bool *done);
void      nvs_config_erase(void);