#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "power_manager.h"
#include "nvs_config.h"
#include "sleep_manager.h"
#include "bme280.h"
#include "pms7003.h"
#include "sx1278.h"
#include "lora_protocol.h"
#include "sd_logger.h"
#include "ds2782.h"
#include "bme680_bsec.h"

#define NODE_ID  1

// Calibration constants
#define CALIBRATION_HOURS     2.0f
#define CAL_INTERVAL_SECONDS  30
#define CALIBRATION_READINGS  ((int)(CALIBRATION_HOURS * 3600.0f / CAL_INTERVAL_SECONDS))

static const char *TAG  = "AGNI_EDGE";
static sx1278_t    lora;

static void i2c_buses_init(void) {
    i2c_config_t c0 = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = GPIO_NUM_8,
        .scl_io_num       = GPIO_NUM_9,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &c0);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    i2c_config_t c1 = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = GPIO_NUM_41,
        .scl_io_num       = GPIO_NUM_42,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_1, &c1);
    i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
}

// ── Battery read ─────────────────────────────────────────────
static uint8_t read_battery(float *pack_voltage_out) {
    float   cell1_v = 0.0f, cell2_v = 0.0f;
    uint8_t cell1_soc = 0,  cell2_soc = 0;

    if (ds2782_init(I2C_NUM_1, DS2782_ADDR) == ESP_OK)
        cell1_soc = ds2782_get_soc_safe(I2C_NUM_1, DS2782_ADDR, &cell1_v);
    if (ds2782_init(I2C_NUM_0, DS2782_ADDR) == ESP_OK)
        cell2_soc = ds2782_get_soc_safe(I2C_NUM_0, DS2782_ADDR, &cell2_v);

    float pack_voltage = cell1_v + cell2_v;
    if (pack_voltage_out) *pack_voltage_out = pack_voltage;

    uint8_t pack_soc;
    if (cell1_v > 0.0f && cell2_v > 0.0f) {
        int s = (int)((pack_voltage / 8.4f) * 100.0f);
        pack_soc = (uint8_t)(s < 0 ? 0 : s > 100 ? 100 : s);
    } else if (cell1_v > 0.0f) {
        int s = (int)((cell1_v / 4.2f) * 100.0f);
        pack_soc = (uint8_t)(s < 0 ? 0 : s > 100 ? 100 : s);
    } else if (cell2_v > 0.0f) {
        int s = (int)((cell2_v / 4.2f) * 100.0f);
        pack_soc = (uint8_t)(s < 0 ? 0 : s > 100 ? 100 : s);
    } else {
        pack_soc = 0;
    }

    ESP_LOGI(TAG,
             "Battery: %.2fV SOC=%d%% "
             "C1=%.3fV(%d%%) C2=%.3fV(%d%%)",
             pack_voltage, pack_soc,
             cell1_v, cell1_soc,
             cell2_v, cell2_soc);

    return pack_soc;
}

// ── Calibration phase — continuous loop, no deep sleep ───────
static void run_calibration(void) {
    ESP_LOGI(TAG,
             "=== CALIBRATION START: %.1fh / %d readings @ %ds ===",
             (float)CALIBRATION_HOURS,
             CALIBRATION_READINGS,
             CAL_INTERVAL_SECONDS);

    float    cal_sum   = 0.0f;
    int      cal_count = 0;
    int64_t  start_us  = esp_timer_get_time();
    int64_t  duration_us = (int64_t)(CALIBRATION_HOURS * 3600.0f * 1e6f);

    while (esp_timer_get_time() - start_us < duration_us) {

        bme680_data_t bme680 = {0};
        esp_err_t ret = bme680_read(&bme680);

        if (ret == ESP_OK && bme680.gas_valid &&
            bme680.gas_resistance > 0.0f) {

            cal_sum += bme680.gas_resistance;
            cal_count++;
            float mean = cal_sum / (float)cal_count;
            float elapsed_min =
                (float)(esp_timer_get_time() - start_us) / 60e6f;

            ESP_LOGI(TAG,
                     "Cal [%d/%d] Gas=%.2fkΩ Mean=%.2fkΩ "
                     "T=%.1f H=%.1f Elapsed=%.1fmin",
                     cal_count, CALIBRATION_READINGS,
                     bme680.gas_resistance, mean,
                     bme680.temperature, bme680.humidity,
                     elapsed_min);

            // Write cal entry to SD
            power_manager_sd_on();
            if (sd_logger_init() == ESP_OK) {
                sd_cal_entry_t cal = {
                    .boot           = (uint32_t)g_boot_count,
                    .seq            = (uint16_t)cal_count,
                    .node_id        = NODE_ID,
                    .reading_num    = (uint32_t)cal_count,
                    .total_readings = (uint32_t)CALIBRATION_READINGS,
                    .gas_resistance = bme680.gas_resistance,
                    .temperature    = bme680.temperature,
                    .humidity       = bme680.humidity,
                    .pressure       = bme680.pressure,
                    .running_mean   = mean,
                };
                sd_logger_write_cal(&cal, NODE_ID);
                sd_logger_deinit();
            }
            power_manager_sd_off();

        } else {
            ESP_LOGW(TAG, "Cal: BME680 gas invalid — reading skipped");
        }

        // Wait for next reading — RTOS friendly
        vTaskDelay(pdMS_TO_TICKS(CAL_INTERVAL_SECONDS * 1000));
    }

    // Calibration complete
    if (cal_count > 0) {
        g_gas_baseline     = cal_sum / (float)cal_count;
        g_calibration_done = true;
        nvs_config_save_baseline(g_gas_baseline);
        nvs_config_save_calibration_flag(true);
        ESP_LOGI(TAG,
                 "=== CALIBRATION COMPLETE: baseline=%.2fkΩ "
                 "over %d readings ===",
                 g_gas_baseline, cal_count);
    } else {
        ESP_LOGE(TAG, "Calibration: zero valid readings — "
                      "baseline not set, will retry next boot");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== AGNI GUARD Edge Node v2.0 ===");

    // ── 1. NVS ───────────────────────────────────────────────
    nvs_config_init();

    // ── 2. Sleep manager ─────────────────────────────────────
    sleep_manager_init();
    ESP_LOGI(TAG, "Boot #%d | Stage: %s | Seq: %d",
             g_boot_count,
             STAGE_NAMES[g_current_stage],
             g_sequence_number);

    // ── 3. Power on ──────────────────────────────────────────
    power_manager_init();
    power_manager_sensors_on();
    i2c_buses_init();

    // ── 4. Battery ───────────────────────────────────────────
    float   pack_voltage = 0.0f;
    uint8_t pack_soc     = read_battery(&pack_voltage);

    // ── 5. Sensor init ───────────────────────────────────────
    bme280_init();
    esp_err_t bme680_ok = bme680_init();
    pms7003_init();

    // ── 6. Calibration phase (continuous — no sleep, no LoRa)
    if (!g_calibration_done) {
        if (bme680_ok == ESP_OK) {
            run_calibration();
        } else {
            ESP_LOGE(TAG, "BME680 init failed — cannot calibrate");
        }
        // One deep sleep then wake into normal ops
        power_manager_sensors_off();
        sleep_manager_enter_deep_sleep_with_soc(STAGE_NORMAL, pack_soc);
        return;
    }

    // ── 7. LoRa init ─────────────────────────────────────────
    esp_err_t lora_ok = sx1278_init(&lora,
        SX1278_EDGE_SPI,
        SX1278_EDGE_MOSI, SX1278_EDGE_MISO,
        SX1278_EDGE_SCK,  SX1278_EDGE_NSS,
        SX1278_EDGE_RST,  SX1278_EDGE_DIO0);

    // ── 9. Read sensors ──────────────────────────────────────
    bme280_data_t  bme280 = {0};
    bme680_data_t  bme680 = {0};
    pms7003_data_t pms    = {0};

    bme280_read(&bme280);
    if (bme680_ok == ESP_OK) {
        bme680_read(&bme680);
    }
    pms7003_read(&pms);

    ESP_LOGI(TAG,
             "BME680: T=%.2f H=%.2f P=%.2f Gas=%.2fkΩ valid=%d",
             bme680.temperature, bme680.humidity,
             bme680.pressure, bme680.gas_resistance,
             bme680.gas_valid);
    ESP_LOGI(TAG,
             "BME280: T=%.2f H=%.2f P=%.2f",
             bme280.temperature, bme280.humidity,
             bme280.pressure);
    ESP_LOGI(TAG,
             "PMS: PM2.5=%d PM10=%d µg/m³",
             pms.pm2_5, pms.pm10);

    // ── 10. Load gas baseline ─────────────────────────────────
    if (g_gas_baseline == 0.0f) {
        nvs_config_load_baseline(&g_gas_baseline);
    }

    // ── 11. EMA baseline update — STAGE_NORMAL only ──────────
    if (bme680_ok == ESP_OK &&
        bme680.gas_valid &&
        bme680.gas_resistance > 0.0f &&
        g_current_stage == STAGE_NORMAL) {

        g_gas_baseline = (g_gas_baseline * 0.95f) +
                         (bme680.gas_resistance * 0.05f);
        nvs_config_save_baseline(g_gas_baseline);
        ESP_LOGI(TAG, "Gas baseline (EMA): %.2f kΩ", g_gas_baseline);
    } else {
        ESP_LOGI(TAG, "Gas baseline (held): %.2f kΩ", g_gas_baseline);
    }

    // ── 12. SD log ───────────────────────────────────────────
    power_manager_sd_on();
    if (sd_logger_init() == ESP_OK) {
        sd_log_entry_t entry = {
            .boot            = (uint32_t)g_boot_count,
            .seq             = g_sequence_number,
            .node_id         = NODE_ID,
            .bme680_temp     = bme680.temperature,
            .bme680_humidity = bme680.humidity,
            .bme680_pressure = bme680.pressure,
            .bme680_gas      = bme680.gas_resistance,
            .bme280          = bme280,
            .pms             = pms,
            .soc             = pack_soc,
            .battery_voltage = pack_voltage,
            .stage           = g_current_stage,
        };
        sd_logger_write(&entry, NODE_ID);
        sd_logger_deinit();
    }
    power_manager_sd_off();

    // ── 13. Transmit + wait ACK ──────────────────────────────
    fire_stage_t new_stage = g_current_stage;

    if (lora_ok == ESP_OK) {
        lora_packet_t pkt = {
            .bme680_temp     = bme680.temperature,
            .bme680_humidity = bme680.humidity,
            .bme680_pressure = bme680.pressure,
            .bme680_gas      = bme680.gas_resistance,
            .bme280_temp     = bme280.temperature,
            .bme280_humidity = bme280.humidity,
            .bme280_pressure = bme280.pressure,
            .pm2_5           = pms.pm2_5,
            .pm10            = pms.pm10,
            .battery_soc     = pack_soc,
            .battery_voltage = pack_voltage,
            .gas_baseline    = g_gas_baseline,
            .sequence        = g_sequence_number,
        };

        if (lora_transmit(&lora, &pkt, NODE_ID) == ESP_OK) {
            lora_wait_ack(&lora, NODE_ID, &new_stage);
        }
    }

    // ── 14. Power off + sleep ─────────────────────────────────
    power_manager_sensors_off();
    if (lora_ok == ESP_OK) sx1278_sleep(&lora);
    sleep_manager_enter_deep_sleep_with_soc(new_stage, pack_soc);
}
