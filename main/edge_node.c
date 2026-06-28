#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
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

static const char *TAG  = "AGNI_EDGE";
static sx1278_t    lora;

static void i2c_buses_init(void) {
    // Bus 0: BME280 + DS2782 Cell2
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

    // Bus 1: BME680 + DS2782 Cell1
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
    float   cell1_v = 0.0f, cell2_v = 0.0f;
    uint8_t cell1_soc = 0,  cell2_soc = 0;

    if (ds2782_init(I2C_NUM_1, DS2782_ADDR) == ESP_OK) {
        cell1_soc = ds2782_get_soc_safe(I2C_NUM_1,
                                         DS2782_ADDR,
                                         &cell1_v);
    }
    if (ds2782_init(I2C_NUM_0, DS2782_ADDR) == ESP_OK) {
        cell2_soc = ds2782_get_soc_safe(I2C_NUM_0,
                                         DS2782_ADDR,
                                         &cell2_v);
    }

    float   pack_voltage = cell1_v + cell2_v;
    // Voltage-based SOC: full=8.4V (4.2V×2), single cell full=4.2V.
    // DS2782 SOC register unreliable (Cell2 reports 0% at 2.655V).
    uint8_t pack_soc;
    if (cell1_v > 0.0f && cell2_v > 0.0f) {
        int soc = (int)((pack_voltage / 8.4f) * 100.0f);
        pack_soc = (uint8_t)(soc < 0 ? 0 : soc > 100 ? 100 : soc);
    } else if (cell1_v > 0.0f) {
        int soc = (int)((cell1_v / 4.2f) * 100.0f);
        pack_soc = (uint8_t)(soc < 0 ? 0 : soc > 100 ? 100 : soc);
    } else if (cell2_v > 0.0f) {
        int soc = (int)((cell2_v / 4.2f) * 100.0f);
        pack_soc = (uint8_t)(soc < 0 ? 0 : soc > 100 ? 100 : soc);
    } else {
        pack_soc = 0;
    }

    ESP_LOGI(TAG,
             "Battery: %.2fV SOC=%d%% "
             "C1=%.3fV(%d%%) C2=%.3fV(%d%%)",
             pack_voltage, pack_soc,
             cell1_v, cell1_soc,
             cell2_v, cell2_soc);

    // ── 5. Sensor init ───────────────────────────────────────
    bme280_init();
    esp_err_t bme680_ok = bme680_bsec_init();
    pms7003_init();

    // ── 6. LoRa init ─────────────────────────────────────────
    esp_err_t lora_ok = sx1278_init(&lora,
        SX1278_EDGE_SPI,
        SX1278_EDGE_MOSI, SX1278_EDGE_MISO,
        SX1278_EDGE_SCK,  SX1278_EDGE_NSS,
        SX1278_EDGE_RST,  SX1278_EDGE_DIO0);

    // ── 7. Read sensors ──────────────────────────────────────
    bme280_data_t  bme280 = {0};
    bme680_data_t  bme680 = {0};
    pms7003_data_t pms    = {0};

    bme280_read(&bme280);
    if (bme680_ok == ESP_OK) {
        bme680_bsec_read(&bme680);
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

    // ── 8. Calibration or Normal operation ───────────────────
    if (!g_calibration_done) {
        // ── CALIBRATION PHASE ─────────────────────────────────
        // No LoRa. Accumulate BME680 gas readings, write to SD.
        // EN button press resets g_cal_sum/g_cal_count → restarts.
        ESP_LOGI(TAG,
                 "=== CALIBRATION %lu/%d (%.1fh/%.1fh) ===",
                 (unsigned long)g_cal_count + 1,
                 CALIBRATION_READINGS,
                 (float)(g_cal_count) / 60.0f,
                 (float)CALIBRATION_HOURS);

        if (bme680_ok == ESP_OK &&
            bme680.gas_valid &&
            bme680.gas_resistance > 0.0f) {

            g_cal_sum += bme680.gas_resistance;
            g_cal_count++;
            float running_mean = g_cal_sum / (float)g_cal_count;

            power_manager_sd_on();
            if (sd_logger_init() == ESP_OK) {
                sd_cal_entry_t cal = {
                    .boot           = (uint32_t)g_boot_count,
                    .seq            = g_sequence_number,
                    .node_id        = NODE_ID,
                    .reading_num    = g_cal_count,
                    .total_readings = CALIBRATION_READINGS,
                    .gas_resistance = bme680.gas_resistance,
                    .temperature    = bme680.temperature,
                    .humidity       = bme680.humidity,
                    .pressure       = bme680.pressure,
                    .running_mean   = running_mean,
                };
                sd_logger_write_cal(&cal, NODE_ID);
                sd_logger_deinit();
            }
            power_manager_sd_off();

            if (g_cal_count >= (uint32_t)CALIBRATION_READINGS) {
                g_gas_baseline     = running_mean;
                g_calibration_done = true;
                nvs_config_save_baseline(g_gas_baseline);
                nvs_config_save_calibration_flag(true);
                ESP_LOGI(TAG,
                         "=== CALIBRATION COMPLETE ==="
                         " baseline=%.2f kΩ over %d readings ===",
                         g_gas_baseline, CALIBRATION_READINGS);
            }
        } else {
            ESP_LOGW(TAG, "Cal: BME680 gas invalid — reading skipped");
        }

        power_manager_sensors_off();
        if (lora_ok == ESP_OK) sx1278_sleep(&lora);
        sleep_manager_enter_deep_sleep_with_soc(STAGE_NORMAL, pack_soc);
        return;
    }

    // ── 9. Normal operation — update gas baseline (EMA) ──────
    if (g_gas_baseline == 0.0f) {
        nvs_config_load_baseline(&g_gas_baseline);
    }
    if (bme680.gas_valid && bme680.gas_resistance > 0.0f) {
        g_gas_baseline = (g_gas_baseline * 0.9f) +
                         (bme680.gas_resistance * 0.1f);
    }
    ESP_LOGI(TAG, "Gas baseline: %.2f kΩ", g_gas_baseline);

    // ── 10. SD log ───────────────────────────────────────────
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

    // ── 11. Transmit + wait ACK ──────────────────────────────
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

    // ── 12. Save gas baseline ────────────────────────────────
    if (g_gas_baseline > 0.0f) {
        nvs_config_save_baseline(g_gas_baseline);
    }

    // ── 13. Power off ────────────────────────────────────────
    power_manager_sensors_off();
    if (lora_ok == ESP_OK) sx1278_sleep(&lora);

    // ── 14. Sleep ────────────────────────────────────────────
    sleep_manager_enter_deep_sleep_with_soc(new_stage, pack_soc);
}