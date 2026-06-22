#include "ds2782.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DS2782";

// ============================================================
// I2C helpers
// ============================================================

static esp_err_t ds2782_read_reg(i2c_port_t port, uint8_t addr,
                                  uint8_t reg, uint8_t *buf,
                                  size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ds2782_write_reg(i2c_port_t port, uint8_t addr,
                                   uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE,
                          true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function command: write FCmd to 0xFE (page 26)
static esp_err_t ds2782_func_cmd(i2c_port_t port, uint8_t addr,
                                  uint8_t fcmd) {
    return ds2782_write_reg(port, addr, DS2782_REG_FUNC_CMD, fcmd);
}

// ============================================================
// EEPROM programming — Samsung 30Q at Rsns=20mΩ
// ============================================================

static esp_err_t ds2782_program_eeprom(i2c_port_t port,
                                        uint8_t addr) {
    ESP_LOGI(TAG, "Programming EEPROM (Samsung 30Q, Rsns=20mΩ)");

    // Write all parameters to shadow RAM (Parameter EEPROM block 1)
    // These values are calculated in ds2782.h defines above

    // AC: Aging Capacity — 2500mAh / 0.3125mAh per LSB = 8000 = 0x1F40
    ds2782_write_reg(port, addr,
                     DS2782_REG_AC_MSB, DS2782_EEPROM_AC_MSB);
    ds2782_write_reg(port, addr,
                     DS2782_REG_AC_LSB, DS2782_EEPROM_AC_LSB);

    // VCHG: Charge voltage 4.15V = 212 = 0xD4
    ds2782_write_reg(port, addr,
                     DS2782_REG_VCHG, DS2782_EEPROM_VCHG);

    // IMIN: Min charge current 100mA = 40 = 0x28
    ds2782_write_reg(port, addr,
                     DS2782_REG_IMIN, DS2782_EEPROM_IMIN);

    // VAE: Active empty voltage 3.0V = 153 = 0x99
    ds2782_write_reg(port, addr,
                     DS2782_REG_VAE, DS2782_EEPROM_VAE);

    // IAE: Active empty current 500mA = 50 = 0x32
    ds2782_write_reg(port, addr,
                     DS2782_REG_IAE, DS2782_EEPROM_IAE);

    // RSNSP: Sense resistor prime 20mΩ = 50S = 0x32
    ds2782_write_reg(port, addr,
                     DS2782_REG_RSNSP, DS2782_EEPROM_RSNSP);

    // Copy shadow RAM → EEPROM Block 1 (FCmd = 0x44, Table 5)
    ds2782_func_cmd(port, addr, DS2782_CMD_COPY_BLOCK1);

    // Wait for EEPROM write — tEEC = 2ms typ, 10ms max (page 27)
    vTaskDelay(pdMS_TO_TICKS(15));

    // Verify by reading back RSNSP
    uint8_t verify = 0;
    ds2782_read_reg(port, addr, DS2782_REG_RSNSP, &verify, 1);

    if (verify == DS2782_EEPROM_RSNSP) {
        ESP_LOGI(TAG, "EEPROM programmed and verified OK");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "EEPROM verify FAILED — "
                      "RSNSP=0x%02X (expected 0x%02X)",
                 verify, DS2782_EEPROM_RSNSP);
        return ESP_FAIL;
    }
}

// ============================================================
// Init
// ============================================================

esp_err_t ds2782_init(i2c_port_t port, uint8_t addr) {
    // Test communication via STATUS register
    uint8_t status = 0;
    esp_err_t ret = ds2782_read_reg(port, addr,
                                     DS2782_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS2782 @ 0x%02X not found on I2C_%d",
                 addr, port);
        return ret;
    }

    ESP_LOGI(TAG, "DS2782 @ 0x%02X found — STATUS=0x%02X",
             addr, status);

    // ── Step 1: Clear PORF ───────────────────────────────────
    // PORF is set on every power-up (page 19)
    // Must be cleared by user before SOC algorithm is reliable
    // Only UVF (bit2) and PORF (bit1) are user writable
    if (status & DS2782_STATUS_PORF) {
        uint8_t clear = status & ~DS2782_STATUS_PORF;
        ds2782_write_reg(port, addr, DS2782_REG_STATUS, clear);
        ESP_LOGI(TAG, "PORF cleared");
    } else {
        ESP_LOGI(TAG, "PORF already clear");
    }

    // ── Step 2: Check if EEPROM is programmed ───────────────
    // Read RSNSP — if 0x00 then EEPROM was never programmed
    uint8_t rsnsp = 0;
    ds2782_read_reg(port, addr, DS2782_REG_RSNSP, &rsnsp, 1);

    if (rsnsp == 0x00) {
        ESP_LOGW(TAG, "RSNSP=0x00 — EEPROM not programmed");
        ret = ds2782_program_eeprom(port, addr);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "EEPROM programming failed");
            return ret;
        }
        // Recall programmed values into shadow RAM for immediate use
        ds2782_func_cmd(port, addr, DS2782_CMD_RECALL_BLOCK1);
        vTaskDelay(pdMS_TO_TICKS(5));
    } else {
        ESP_LOGI(TAG, "EEPROM OK — RSNSP=0x%02X (Rsns=%.0fmΩ)",
                 rsnsp, 1000.0f / (float)rsnsp);
    }

    // ── Step 3: Log learning status ─────────────────────────
    // LEARNF=1 means learning cycle IN PROGRESS (page 16)
    // LEARNF=0 after CHGTF=1 means learning COMPLETE
    if (status & DS2782_STATUS_LEARNF) {
        ESP_LOGW(TAG, "LEARNF set — learning cycle in progress");
        ESP_LOGW(TAG, "SOC from voltage curve until learning done");
    }
    if (status & DS2782_STATUS_CHGTF) {
        ESP_LOGI(TAG, "CHGTF set — battery was fully charged");
    }

    return ESP_OK;
}

// ============================================================
// Read all measurements
// ============================================================

esp_err_t ds2782_read(i2c_port_t port, uint8_t addr,
                      ds2782_data_t *data) {
    uint8_t buf[2];
    uint8_t status = 0;

    // ── STATUS ───────────────────────────────────────────────
    ds2782_read_reg(port, addr, DS2782_REG_STATUS, &status, 1);
    data->porf_was_set   = (status & DS2782_STATUS_PORF)  != 0;
    // learning_done: LEARNF clear AND at least one CHGTF detected
    data->learning_done  = !(status & DS2782_STATUS_LEARNF);

    // ── Voltage: 0x0C-0x0D ──────────────────────────────────
    // 11-bit value, left-aligned in 16 bits
    // Bits [15:5] = voltage, bits [4:0] = reserved
    // Shift right 5 to get 11-bit value
    // LSB = 4.88mV (DS2782_VOLT_LSB_V)
    if (ds2782_read_reg(port, addr,
                        DS2782_REG_VOLT_MSB, buf, 2) == ESP_OK) {
        int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
        raw >>= 5;
        data->voltage = (float)raw * DS2782_VOLT_LSB_V;
    }

    // ── Temperature: 0x0A-0x0B ──────────────────────────────
    // 11-bit value, left-aligned in 16 bits
    // Shift right 5, LSB = 0.125°C (DS2782_TEMP_LSB_C)
    // Two's complement for negative temperatures
    if (ds2782_read_reg(port, addr,
                        DS2782_REG_TEMP_MSB, buf, 2) == ESP_OK) {
        int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
        raw >>= 5;
        data->temperature = (float)raw * DS2782_TEMP_LSB_C;
    }

    // ── Instantaneous Current: 0x0E-0x0F ────────────────────
    // Full 16-bit signed, NO shift (Figure 6, page 8)
    // LSB = 1.5625µV / Rsns = 1.5625µV / 0.020Ω = 78.125µA
    // Positive = charge, Negative = discharge
    if (ds2782_read_reg(port, addr,
                        DS2782_REG_CURR_MSB, buf, 2) == ESP_OK) {
        int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
        data->current = (float)raw * DS2782_CURR_LSB_MA;
    }

    // ── Average Current: 0x08-0x09 IAVG ────────────────────
    // Full 16-bit signed, NO shift (Figure 7, page 9)
    // Updated every 28 seconds (average of 8 readings)
    // Same LSB as instantaneous current
    if (ds2782_read_reg(port, addr,
                        DS2782_REG_IAVG_MSB, buf, 2) == ESP_OK) {
        int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
        data->avg_current = (float)raw * DS2782_CURR_LSB_MA;
    }

    // ── SOC%: 0x06 RARC ─────────────────────────────────────
    // Direct percentage 0-100, 1% per LSB (Figure 16, page 18)
    // Only valid after EEPROM programmed and learning complete
    uint8_t rarc = 0;
    if (ds2782_read_reg(port, addr,
                        DS2782_REG_RARC, &rarc, 1) == ESP_OK) {
        data->soc_percent = rarc;
    }

    // ── Remaining Capacity: 0x02-0x03 RAAC ──────────────────
    // 16-bit unsigned, LSB = 1.6mAh (Figure 14, page 17)
    if (ds2782_read_reg(port, addr,
                        DS2782_REG_RAAC_MSB, buf, 2) == ESP_OK) {
        uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
        data->capacity_mah = (uint16_t)((float)raw *
                                         DS2782_RAAC_LSB_MAH);
    }

    ESP_LOGI(TAG, "V=%.3fV I=%.1fmA Iavg=%.1fmA "
                  "T=%.1f°C SOC=%d%% Cap=%dmAh [%s]",
             data->voltage,
             data->current,
             data->avg_current,
             data->temperature,
             data->soc_percent,
             data->capacity_mah,
             data->learning_done ? "LEARNED" : "ESTIMATING");

    return ESP_OK;
}

// ============================================================
// Safe SOC — uses RARC if learning done, voltage curve if not
// ============================================================

uint8_t ds2782_get_soc_safe(i2c_port_t port, uint8_t addr,
                             float *voltage_out) {
    uint8_t buf[2];
    uint8_t status = 0;

    // Read voltage first — always needed
    float voltage = 0.0f;
    if (ds2782_read_reg(port, addr,
                        DS2782_REG_VOLT_MSB, buf, 2) == ESP_OK) {
        int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
        raw >>= 5;
        voltage = (float)raw * DS2782_VOLT_LSB_V;
    }
    if (voltage_out) *voltage_out = voltage;

    // Check if learning is complete
    ds2782_read_reg(port, addr, DS2782_REG_STATUS, &status, 1);
    bool learning_done = !(status & DS2782_STATUS_LEARNF) &&
                         !(status & DS2782_STATUS_PORF);

    if (learning_done) {
        // Trust RARC directly
        uint8_t rarc = 0;
        ds2782_read_reg(port, addr, DS2782_REG_RARC, &rarc, 1);
        ESP_LOGD(TAG, "SOC from RARC: %d%%", rarc);
        return rarc;
    }

    // Fallback: Samsung 30Q voltage-based SOC lookup
    // Based on typical discharge curve at ~500mA load
    uint8_t soc;
    if      (voltage >= 4.15f) soc = 100;
    else if (voltage >= 4.05f) soc = 90;
    else if (voltage >= 3.95f) soc = 80;
    else if (voltage >= 3.85f) soc = 70;
    else if (voltage >= 3.75f) soc = 60;
    else if (voltage >= 3.65f) soc = 50;
    else if (voltage >= 3.55f) soc = 40;
    else if (voltage >= 3.45f) soc = 30;
    else if (voltage >= 3.30f) soc = 20;
    else if (voltage >= 3.10f) soc = 10;
    else                        soc = 5;

    ESP_LOGW(TAG, "SOC from voltage curve: %.3fV → %d%%",
             voltage, soc);
    return soc;
}

// ============================================================
// Raw diagnostic dump — call to debug inaccurate readings
// ============================================================

void ds2782_raw_dump(i2c_port_t port, uint8_t addr) {
    uint8_t buf[2];
    uint8_t val;

    ESP_LOGI(TAG, "╔══════════════════════════════════╗");
    ESP_LOGI(TAG, "║     DS2782 Raw Register Dump     ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════╝");

    // STATUS
    ds2782_read_reg(port, addr, DS2782_REG_STATUS, &val, 1);
    ESP_LOGI(TAG, "STATUS  0x01 = 0x%02X | "
                  "CHGTF=%d AEF=%d SEF=%d LEARNF=%d UVF=%d PORF=%d",
             val,
             (val & DS2782_STATUS_CHGTF)  ? 1 : 0,
             (val & DS2782_STATUS_AEF)    ? 1 : 0,
             (val & DS2782_STATUS_SEF)    ? 1 : 0,
             (val & DS2782_STATUS_LEARNF) ? 1 : 0,
             (val & DS2782_STATUS_UVF)    ? 1 : 0,
             (val & DS2782_STATUS_PORF)   ? 1 : 0);

    // RARC — direct SOC
    ds2782_read_reg(port, addr, DS2782_REG_RARC, &val, 1);
    ESP_LOGI(TAG, "RARC    0x06 = %d%%", val);

    // IAVG — average current
    ds2782_read_reg(port, addr, DS2782_REG_IAVG_MSB, buf, 2);
    int16_t iavg = ((int16_t)buf[0] << 8) | buf[1];
    ESP_LOGI(TAG, "IAVG    0x08 = 0x%02X%02X → %.1fmA",
             buf[0], buf[1], (float)iavg * DS2782_CURR_LSB_MA);

    // TEMP
    ds2782_read_reg(port, addr, DS2782_REG_TEMP_MSB, buf, 2);
    int16_t temp = (((int16_t)buf[0] << 8) | buf[1]) >> 5;
    ESP_LOGI(TAG, "TEMP    0x0A = 0x%02X%02X → %.2f°C",
             buf[0], buf[1], (float)temp * DS2782_TEMP_LSB_C);

    // VOLT
    ds2782_read_reg(port, addr, DS2782_REG_VOLT_MSB, buf, 2);
    uint16_t vraw = ((uint16_t)buf[0] << 8) | buf[1];
    ESP_LOGI(TAG, "VOLT    0x0C = 0x%02X%02X raw=0x%04X >>5=%d → %.3fV",
             buf[0], buf[1], vraw,
             vraw >> 5,
             (float)(vraw >> 5) * DS2782_VOLT_LSB_V);

    // CURR
    ds2782_read_reg(port, addr, DS2782_REG_CURR_MSB, buf, 2);
    int16_t curr = ((int16_t)buf[0] << 8) | buf[1];
    ESP_LOGI(TAG, "CURR    0x0E = 0x%02X%02X → %.1fmA",
             buf[0], buf[1], (float)curr * DS2782_CURR_LSB_MA);

    // ACR
    ds2782_read_reg(port, addr, DS2782_REG_ACR_MSB, buf, 2);
    uint16_t acr = ((uint16_t)buf[0] << 8) | buf[1];
    ESP_LOGI(TAG, "ACR     0x10 = 0x%04X", acr);

    // RAAC — remaining capacity
    ds2782_read_reg(port, addr, DS2782_REG_RAAC_MSB, buf, 2);
    uint16_t raac = ((uint16_t)buf[0] << 8) | buf[1];
    ESP_LOGI(TAG, "RAAC    0x02 = 0x%04X → %dmAh",
             raac, (uint16_t)((float)raac * DS2782_RAAC_LSB_MAH));

    // EEPROM parameters
    ESP_LOGI(TAG, "── EEPROM Parameters ──────────────");

    ds2782_read_reg(port, addr, DS2782_REG_RSNSP, &val, 1);
    ESP_LOGI(TAG, "RSNSP   0x69 = 0x%02X = %dS → Rsns=%.1fmΩ",
             val, val, val > 0 ? 1000.0f / (float)val : 0.0f);

    ds2782_read_reg(port, addr, DS2782_REG_VCHG, &val, 1);
    ESP_LOGI(TAG, "VCHG    0x64 = 0x%02X → %.0fmV",
             val, (float)val * 19.52f);

    ds2782_read_reg(port, addr, DS2782_REG_VAE, &val, 1);
    ESP_LOGI(TAG, "VAE     0x66 = 0x%02X → %.0fmV",
             val, (float)val * 19.52f);

    ds2782_read_reg(port, addr, DS2782_REG_IMIN, &val, 1);
    ESP_LOGI(TAG, "IMIN    0x65 = 0x%02X → %.0fmA (at 20mΩ)",
             val, (float)val * 2.5f);

    ds2782_read_reg(port, addr, DS2782_REG_IAE, &val, 1);
    ESP_LOGI(TAG, "IAE     0x67 = 0x%02X → %.0fmA (at 20mΩ)",
             val, (float)val * 10.0f);

    ds2782_read_reg(port, addr, DS2782_REG_AC_MSB, buf, 2);
    uint16_t ac = ((uint16_t)buf[0] << 8) | buf[1];
    ESP_LOGI(TAG, "AC      0x62 = 0x%04X → %.0fmAh",
             ac, (float)ac * 0.3125f);

    ESP_LOGI(TAG, "══════════════════════════════════");
}