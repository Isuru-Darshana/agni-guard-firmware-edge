#include "esp_log.h"
#include "nvs_flash.h"
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

    // ── 3. Power on sensors ──────────────────────────────────
    power_manager_init();
    power_manager_sensors_on();

    // ── 4. Battery measurement (DS2782) ──────────────────────
    // Cell 1: I2C_NUM_1 — SDA=GPIO41, SCL=GPIO42
    // Cell 2: I2C_NUM_0 — SDA=GPIO8,  SCL=GPIO9
    // DS2782 addr=0x34, BME280=0x76, BME680=0x77 — no conflict
    float   cell1_v   = 0.0f, cell2_v   = 0.0f;
    uint8_t cell1_soc = 0,    cell2_soc = 0;

    if (ds2782_init(I2C_NUM_1, DS2782_ADDR) == ESP_OK) {
        cell1_soc = ds2782_get_soc_safe(I2C_NUM_1,
                                         DS2782_ADDR, &cell1_v);
    }
    if (ds2782_init(I2C_NUM_0, DS2782_ADDR) == ESP_OK) {
        cell2_soc = ds2782_get_soc_safe(I2C_NUM_0,
                                         DS2782_ADDR, &cell2_v);
    }

    float   pack_voltage = cell1_v + cell2_v;
    uint8_t pack_soc     = (cell1_soc < cell2_soc) ?
                            cell1_soc : cell2_soc;

    ESP_LOGI(TAG,
             "Battery: Pack=%.2fV SOC=%d%% "
             "Cell1=%.3fV(%d%%) Cell2=%.3fV(%d%%)",
             pack_voltage, pack_soc,
             cell1_v, cell1_soc,
             cell2_v, cell2_soc);

    // ── 5. Sensor init ───────────────────────────────────────
    // BME280 inits I2C_NUM_0 — BME680 shares same bus
    bme280_init();
    bme680_bsec_init();
    pms7003_init();

    // ── 6. LoRa init ─────────────────────────────────────────
    esp_err_t lora_ok = sx1278_init(&lora,
        SX1278_EDGE_SPI,
        SX1278_EDGE_MOSI, SX1278_EDGE_MISO,
        SX1278_EDGE_SCK,  SX1278_EDGE_NSS,
        SX1278_EDGE_RST,  SX1278_EDGE_DIO0);

    // ── 7. Load gas baseline from NVS ────────────────────────
    if (g_gas_baseline == 0.0f) {
        nvs_config_load_baseline(&g_gas_baseline);
    }
    ESP_LOGI(TAG, "Gas baseline: %.2f kΩ", g_gas_baseline);

    // ── 8. Read all sensors ──────────────────────────────────
    bme280_data_t  bme280 = {0};
    bme680_data_t  bme680 = {0};
    pms7003_data_t pms    = {0};

    bme280_read(&bme280);
    bme680_bsec_read(&bme680);
    pms7003_read(&pms);

    // ── 9. Update gas baseline from live BME680 reading ──────
    // Only update if gas measurement is valid and sensor stable
    if (bme680.gas_valid &&
        bme680.gas_resistance > 0.0f &&
        bme680.iaq_accuracy >= 1) {
        // Running average: 90% old + 10% new
        if (g_gas_baseline == 0.0f) {
            g_gas_baseline = bme680.gas_resistance;
        } else {
            g_gas_baseline = (g_gas_baseline * 0.9f) +
                             (bme680.gas_resistance * 0.1f);
        }
        ESP_LOGI(TAG, "Baseline updated: %.2f kΩ",
                 g_gas_baseline);
    }

    // Log IAQ accuracy status
    const char *accuracy_str[] = {
        "UNRELIABLE", "LOW", "MEDIUM", "HIGH"
    };
    ESP_LOGI(TAG,
             "BME680: T=%.2f°C H=%.2f%% P=%.2fhPa "
             "Gas=%.2fkΩ IAQ=%.1f [%s]",
             bme680.temperature, bme680.humidity,
             bme680.pressure, bme680.gas_resistance,
             bme680.iaq,
             accuracy_str[bme680.iaq_accuracy]);

    ESP_LOGI(TAG, "CO2eq=%.1fppm VOCeq=%.3fppm",
             bme680.co2_equivalent, bme680.voc_equivalent);

    ESP_LOGI(TAG, "BME280: T=%.2f°C H=%.2f%% P=%.2fhPa",
             bme280.temperature, bme280.humidity,
             bme280.pressure);

    ESP_LOGI(TAG, "PMS7003: PM1.0=%d PM2.5=%d PM10=%d µg/m³",
             pms.pm1_0, pms.pm2_5, pms.pm10);

    // ── 10. SD card log ──────────────────────────────────────
    power_manager_sd_on();
    if (sd_logger_init() == ESP_OK) {
        sd_log_entry_t entry = {
            .boot             = (uint32_t)g_boot_count,
            .seq              = g_sequence_number,
            .node_id          = NODE_ID,
            .bme680_temp      = bme680.temperature,
            .bme680_humidity  = bme680.humidity,
            .bme680_pressure  = bme680.pressure,
            .bme680_gas       = bme680.gas_resistance,
            .bme280           = bme280,
            .pms              = pms,
            .soc              = pack_soc,
            .battery_voltage  = pack_voltage,
            .stage            = g_current_stage,
        };
        sd_logger_write(&entry, NODE_ID);
        sd_logger_deinit();
    }
    power_manager_sd_off();

    // ── 11. Transmit + wait ACK ──────────────────────────────
    fire_stage_t new_stage = g_current_stage;

    if (lora_ok == ESP_OK) {
        lora_packet_t pkt = {
            // BME680 — real data now
            .bme680_temp      = bme680.temperature,
            .bme680_humidity  = bme680.humidity,
            .bme680_pressure  = bme680.pressure,
            .bme680_gas       = bme680.gas_resistance,

            // BME280
            .bme280_temp      = bme280.temperature,
            .bme280_humidity  = bme280.humidity,
            .bme280_pressure  = bme280.pressure,

            // PMS7003
            .pm2_5            = pms.pm2_5,
            .pm10             = pms.pm10,

            // Battery
            .battery_soc      = pack_soc,
            .battery_voltage  = pack_voltage,

            // Calibration
            .gas_baseline     = g_gas_baseline,
            .sequence         = g_sequence_number,
        };

        if (lora_transmit(&lora, &pkt, NODE_ID) == ESP_OK) {
            lora_wait_ack(&lora, NODE_ID, &new_stage);
        }
    }

    // ── 12. Save BSEC calibration state before sleep ─────────
    // Critical — preserves IAQ calibration across deep sleep
    bme680_bsec_save_state();

    // Also save updated gas baseline to NVS
    if (g_gas_baseline > 0.0f) {
        nvs_config_save_baseline(g_gas_baseline);
    }

    // ── 13. Power off ─────────────────────────────────────────
    power_manager_sensors_off();
    if (lora_ok == ESP_OK) sx1278_sleep(&lora);

    // ── 14. Deep sleep with battery protection ────────────────
    sleep_manager_enter_deep_sleep_with_soc(new_stage, pack_soc);
}