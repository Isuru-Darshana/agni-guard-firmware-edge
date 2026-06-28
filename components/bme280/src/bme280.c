#include "bme280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BME280";

// ── Compensation coefficients ──────────────────────────────
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;
static int32_t  t_fine;

// ── I2C helpers ────────────────────────────────────────────
static esp_err_t bme280_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BME280_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bme280_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BME280_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ── Compensation coefficients read ─────────────────────────
// Per datasheet Table 16 section 4.2.2
static esp_err_t bme280_read_compensation(void) {
    uint8_t buf[24] = {0};

    // Temperature + pressure: 0x88 to 0x9F (24 bytes)
    esp_err_t ret = bme280_read_reg(0x88, buf, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "T/P calibration read failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    dig_T1 = (uint16_t)((buf[1]  << 8) | buf[0]);
    dig_T2 = (int16_t) ((buf[3]  << 8) | buf[2]);
    dig_T3 = (int16_t) ((buf[5]  << 8) | buf[4]);
    dig_P1 = (uint16_t)((buf[7]  << 8) | buf[6]);
    dig_P2 = (int16_t) ((buf[9]  << 8) | buf[8]);
    dig_P3 = (int16_t) ((buf[11] << 8) | buf[10]);
    dig_P4 = (int16_t) ((buf[13] << 8) | buf[12]);
    dig_P5 = (int16_t) ((buf[15] << 8) | buf[14]);
    dig_P6 = (int16_t) ((buf[17] << 8) | buf[16]);
    dig_P7 = (int16_t) ((buf[19] << 8) | buf[18]);
    dig_P8 = (int16_t) ((buf[21] << 8) | buf[20]);
    dig_P9 = (int16_t) ((buf[23] << 8) | buf[22]);

    ESP_LOGI(TAG, "Calib: T1=%u T2=%d T3=%d",
             dig_T1, dig_T2, dig_T3);

    // dig_H1 at 0xA1 (separate from main block per Table 16)
    bme280_read_reg(0xA1, &dig_H1, 1);

    // Humidity: 0xE1 to 0xE7 (7 bytes)
    uint8_t hbuf[7] = {0};
    bme280_read_reg(0xE1, hbuf, 7);

    dig_H2 = (int16_t)((hbuf[1] << 8) | hbuf[0]);         // 0xE2/0xE1

    dig_H3 = hbuf[2];                                       // 0xE3

    // dig_H4[11:4] from 0xE4, dig_H4[3:0] from 0xE5[3:0]
    dig_H4 = (int16_t)(((int8_t)hbuf[3] << 4) | (hbuf[4] & 0x0F));

    // dig_H5[3:0] from 0xE5[7:4], dig_H5[11:4] from 0xE6
    dig_H5 = (int16_t)(((int8_t)hbuf[5] << 4) | (hbuf[4] >> 4));

    dig_H6 = (int8_t)hbuf[6];                              // 0xE7
    return ESP_OK;
}

// ── Bosch compensation formulas (from datasheet section 4.2.3) ─
static float compensate_temperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
                    ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                    ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}

static float compensate_pressure(int32_t adc_P) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 += (var1 * (int64_t)dig_P5) << 17;
    var2 += (int64_t)dig_P4 << 35;
    var1  = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
            ((var1 * (int64_t)dig_P2) << 12);
    var1  = (((int64_t)1 << 47) + var1) * (int64_t)dig_P1 >> 33;
    if (var1 == 0) return 0.0f;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);
    return (float)(uint32_t)p / 25600.0f;  // Pa → hPa
}

static float compensate_humidity(int32_t adc_H) {
    // Datasheet s4.2.3: one expression — original v (t_fine-76800) used
    // throughout RHS for dig_H5, dig_H6, dig_H3.  Save before overwriting.
    // Precedence trap: x * EXPR >> 14 = (x*EXPR)>>14 — overflows int32.
    // Must be x * (EXPR >> 14) per datasheet parenthesisation.
    int32_t v = t_fine - 76800;
    int32_t x = (((adc_H << 14) - ((int32_t)dig_H4 << 20) -
                   ((int32_t)dig_H5 * v)) + 16384) >> 15;
    v = x * (((((((v * (int32_t)dig_H6) >> 10) *
                 (((v * (int32_t)dig_H3) >> 11) + 32768)) >> 10) +
                2097152) * (int32_t)dig_H2 + 8192) >> 14);
    v -= ((((v >> 15) * (v >> 15)) >> 7) * (int32_t)dig_H1) >> 4;
    if (v < 0)         v = 0;
    if (v > 419430400) v = 419430400;
    return (float)(v >> 12) / 1024.0f;
}

// ── Init ───────────────────────────────────────────────────
esp_err_t bme280_init(void) {
    // I2C bus already configured by i2c_buses_init() in edge_node.c
    // Wait for sensor startup (datasheet: 2ms after power on)
    vTaskDelay(pdMS_TO_TICKS(5));

    // Verify chip ID — must be 0x60 for BME280 (not 0x58 which is BMP280)
    uint8_t chip_id = 0;
    esp_err_t ret = bme280_read_reg(0xD0, &chip_id, 1);
    if (ret != ESP_OK || chip_id != 0x60) {
        ESP_LOGE(TAG, "BME280 not found! chip_id=0x%02X (expected 0x60)", chip_id);
        return ESP_FAIL;
    }

    // No soft reset — device just powered on, already in clean state.
    // Issuing soft reset causes the BME280 to NACK the data byte mid-write
    // (it resets before ACKing), which can leave the I2C bus dirty and
    // cause the compensation register read to fail silently.

    // Read compensation coefficients
    ret = bme280_read_compensation();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Compensation read failed");
        return ret;
    }

    // Config register 0xF5: no IIR filter, no SPI 3-wire
    // filter[2:0] = 000 (filter off), t_sb irrelevant in forced mode
    bme280_write_reg(0xF5, 0x00);

    // ctrl_hum 0xF2: humidity oversampling ×2
    // osrs_h[2:0] = 010 = oversampling ×2
    // Must be written BEFORE ctrl_meas (per section 5.4.3)
    bme280_write_reg(0xF2, 0x02);

    // Leave in sleep mode after init — forced mode triggered per reading
    // ctrl_meas in sleep: 0x00
    bme280_write_reg(0xF4, 0x00);

    ESP_LOGI(TAG, "BME280 init OK (chip_id=0x60) @ 0x%02X I2C_%d",
             BME280_I2C_ADDR, BME280_I2C_PORT);
    return ESP_OK;
}

// ── Read — forced mode ─────────────────────────────────────
esp_err_t bme280_read(bme280_data_t *data) {
    // Re-write ctrl_hum before each forced mode trigger (required)
    bme280_write_reg(0xF2, 0x02);  // humidity ×2

    // Trigger forced mode measurement
    // ctrl_meas 0xF4:
    //   osrs_t [7:5] = 100 = temp ×8
    //   osrs_p [4:2] = 011 = pressure ×4  (matches your Arduino OS_4X)
    //   mode   [1:0] = 10  = forced mode
    // = 1000 1110 = 0x8E
    bme280_write_reg(0xF4, 0x8E);

    // Wait for measurement to complete
    // Typical time for T×8 + P×4 + H×2 forced mode:
    // t = 1 + [2×8] + [2×4+0.5] + [2×2+0.5] = 1+16+8.5+4.5 = 30ms
    vTaskDelay(pdMS_TO_TICKS(40));

    // Burst read 0xF7–0xFE (8 bytes per section 4)
    uint8_t buf[8];
    esp_err_t ret = bme280_read_reg(0xF7, buf, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Burst read failed");
        return ret;
    }

    // Extract 20-bit ADC values (datasheet section 5.4.7, 5.4.8, 5.4.9)
    int32_t adc_P = ((int32_t)buf[0] << 12) |    // press_msb  0xF7
                    ((int32_t)buf[1] <<  4) |    // press_lsb  0xF8
                    ((int32_t)buf[2] >>  4);     // press_xlsb 0xF9 bits[7:4]

    int32_t adc_T = ((int32_t)buf[3] << 12) |    // temp_msb   0xFA
                    ((int32_t)buf[4] <<  4) |    // temp_lsb   0xFB
                    ((int32_t)buf[5] >>  4);     // temp_xlsb  0xFC bits[7:4]

    int32_t adc_H = ((int32_t)buf[6] <<  8) |    // hum_msb    0xFD
                     (int32_t)buf[7];            // hum_lsb    0xFE

    // Apply compensation formulas
    // Temperature MUST be calculated first — sets t_fine used by P and H
    data->temperature = compensate_temperature(adc_T);
    data->pressure    = compensate_pressure(adc_P);
    data->humidity    = compensate_humidity(adc_H);

    // Sanity check
    if (data->temperature < -40.0f || data->temperature > 85.0f) {
        ESP_LOGW(TAG, "Temperature out of range: %.2f", data->temperature);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "T=%.2f°C  H=%.2f%%  P=%.2fhPa",
             data->temperature, data->humidity, data->pressure);

    return ESP_OK;
}