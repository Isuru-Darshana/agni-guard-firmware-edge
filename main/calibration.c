#include "calibration.h"
#include "nvs_config.h"
#include "sleep_manager.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "CALIB";

// ── Prompt ───────────────────────────────────────────────────
// Returns true  → run calibration
// Returns false → use stored baseline
bool calibration_prompt(void) {
    float stored = 0.0f;
    bool  has_baseline = (nvs_config_load_baseline(&stored) == ESP_OK
                          && stored > 0.0f);

    if (!has_baseline) {
        ESP_LOGW(TAG, "No baseline in NVS — calibration required");
        return true;
    }

    ESP_LOGI(TAG, "Stored baseline: %.2f kΩ", stored);
    ESP_LOGI(TAG, "Recalibrate? [Y/n] (30s timeout):");

    // Flush UART RX
    uart_flush(CONFIG_ESP_CONSOLE_UART_NUM);

    uint32_t start = xTaskGetTickCount();
    char     input[8] = {0};
    int      idx = 0;

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(CALIBRATION_PROMPT_MS)) {
        uint8_t c;
        int read = uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM,
                                   &c, 1, pdMS_TO_TICKS(100));
        if (read == 1) {
            if (c == '\n' || c == '\r') {
                input[idx] = '\0';
                break;
            }
            if (idx < (int)sizeof(input) - 1) {
                input[idx++] = (char)c;
            }
        }
    }

    // Default: No (keep stored baseline)
    if (input[0] == 'Y' || input[0] == 'y') {
        ESP_LOGI(TAG, "User selected: RECALIBRATE");
        return true;
    }

    ESP_LOGI(TAG, "Using stored baseline: %.2f kΩ", stored);
    g_gas_baseline = stored;
    return false;
}

// ── Run calibration ──────────────────────────────────────────
esp_err_t calibration_run(calibration_result_t *result) {
    ESP_LOGI(TAG, "╔══════════════════════════════╗");
    ESP_LOGI(TAG, "║  CALIBRATION — 60 minutes    ║");
    ESP_LOGI(TAG, "║  ALL SENSORS ACTIVE          ║");
    ESP_LOGI(TAG, "╚══════════════════════════════╝");

    memset(result, 0, sizeof(calibration_result_t));

    float    gas_sum    = 0.0f;
    float    gas_min    = 999999.0f;
    float    gas_max    = 0.0f;
    uint32_t pm25_sum   = 0;
    uint32_t readings   = 0;

    uint32_t start     = xTaskGetTickCount();
    uint32_t last_read = 0;

    ESP_LOGI(TAG, "Time(s) | Temp(°C) | Humid(%%) | Gas(kΩ) | PM2.5");
    ESP_LOGI(TAG, "--------|----------|----------|---------|------");

    while ((xTaskGetTickCount() - start) <
           pdMS_TO_TICKS(CALIBRATION_DURATION_MS)) {

        uint32_t now = xTaskGetTickCount();

        if ((now - last_read) >= pdMS_TO_TICKS(CALIBRATION_INTERVAL_MS)) {
            last_read = now;

            // Read BME280
            bme280_data_t bme = {0};
            esp_err_t r1 = bme280_read(&bme);

            // Read PMS7003
            pms7003_data_t pms = {0};
            pms7003_read(&pms);

            if (r1 == ESP_OK && bme.temperature > -40.0f) {
                uint32_t elapsed_s = (xTaskGetTickCount() - start) /
                                     configTICK_RATE_HZ;

                // NOTE: BME680 gas resistance added here once
                // bme680_bsec component is integrated.
                // For now use placeholder — replace with actual
                // bme680_data.gas_resistance when available.
                float gas = g_gas_baseline > 0 ? g_gas_baseline : 100.0f;

                readings++;
                gas_sum  += gas;
                pm25_sum += pms.pm2_5;

                if (gas < gas_min) gas_min = gas;
                if (gas > gas_max) gas_max = gas;

                ESP_LOGI(TAG, "%7lu  | %7.2f  | %7.2f    | %6.2f  | %d",
                         (unsigned long)elapsed_s,
                         bme.temperature,
                         bme.humidity,
                         gas,
                         pms.pm2_5);

                // Progress update every 5 minutes
                if (elapsed_s > 0 && elapsed_s % 300 == 0) {
                    uint32_t remaining = (CALIBRATION_DURATION_MS / 1000)
                                        - elapsed_s;
                    ESP_LOGI(TAG, ">>> %lu seconds remaining (%lu readings)",
                             (unsigned long)remaining,
                             (unsigned long)readings);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (readings == 0) {
        ESP_LOGE(TAG, "No valid readings during calibration");
        result->gas_baseline  = 100.0f;  // fallback
        result->pm25_baseline = 0;
        result->readings      = 0;
        return ESP_FAIL;
    }

    result->gas_baseline  = gas_sum / (float)readings;
    result->pm25_baseline = (uint16_t)(pm25_sum / readings);
    result->readings      = readings;

    ESP_LOGI(TAG, "╔══════════════════════════════════╗");
    ESP_LOGI(TAG, "║    CALIBRATION COMPLETE          ║");
    ESP_LOGI(TAG, "╠══════════════════════════════════╣");
    ESP_LOGI(TAG, "║  Readings:     %lu",
             (unsigned long)readings);
    ESP_LOGI(TAG, "║  Gas Baseline: %.2f kΩ",
             result->gas_baseline);
    ESP_LOGI(TAG, "║  Gas Min:      %.2f kΩ", gas_min);
    ESP_LOGI(TAG, "║  Gas Max:      %.2f kΩ", gas_max);
    ESP_LOGI(TAG, "║  PM2.5 Avg:    %d µg/m³",
             result->pm25_baseline);
    ESP_LOGI(TAG, "╚══════════════════════════════════╝");

    // Save to NVS and RTC memory
    nvs_config_save_baseline(result->gas_baseline);
    nvs_config_save_calibration_flag(true);
    g_gas_baseline     = result->gas_baseline;
    g_calibration_done = true;

    return ESP_OK;
}