#include "bme680_bsec.h"
#include "bme68x_hal.h"
#include "bme68x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BME680";

static struct bme68x_dev bme680_dev;
static i2c_port_t        i2c_port = BME680_I2C_PORT;

// ── Init ─────────────────────────────────────────────────────
esp_err_t bme680_bsec_init(void) {
    bme680_dev.intf     = BME68X_I2C_INTF;
    bme680_dev.read     = bme68x_i2c_read;
    bme680_dev.write    = bme68x_i2c_write;
    bme680_dev.delay_us = bme68x_delay_us;
    bme680_dev.intf_ptr = &i2c_port;
    bme680_dev.amb_temp = 25;

    int8_t rslt = bme68x_init(&bme680_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "BME68x init failed: %d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BME680 init OK");
    return ESP_OK;
}

// ── Read — forced mode, 320°C heater 150ms ───────────────────
esp_err_t bme680_bsec_read(bme680_data_t *data) {
    memset(data, 0, sizeof(bme680_data_t));

    struct bme68x_conf conf = {
        .filter  = BME68X_FILTER_OFF,
        .odr     = BME68X_ODR_NONE,
        .os_hum  = BME68X_OS_2X,
        .os_pres = BME68X_OS_4X,
        .os_temp = BME68X_OS_8X,
    };
    bme68x_set_conf(&conf, &bme680_dev);

    struct bme68x_heatr_conf heatr = {
        .enable     = BME68X_ENABLE,
        .heatr_temp = 320,
        .heatr_dur  = 150,
    };
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr, &bme680_dev);

    bme68x_set_op_mode(BME68X_FORCED_MODE, &bme680_dev);

    uint32_t meas_dur = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme680_dev)
                      + (heatr.heatr_dur * 1000U);
    bme680_dev.delay_us(meas_dur, bme680_dev.intf_ptr);

    struct bme68x_data raw;
    uint8_t n_data = 0;
    int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &raw, &n_data, &bme680_dev);
    if (rslt != BME68X_OK || n_data == 0) {
        ESP_LOGW(TAG, "BME68x read failed: %d", rslt);
        return ESP_FAIL;
    }

    data->temperature    = raw.temperature;
    data->humidity       = raw.humidity;
    data->pressure       = raw.pressure / 100.0f;
    data->gas_resistance = raw.gas_resistance / 1000.0f;
    data->gas_valid      = (raw.status & BME68X_GASM_VALID_MSK) != 0;

    ESP_LOGI(TAG, "T=%.2f°C H=%.2f%% P=%.2fhPa Gas=%.2fkΩ [%s]",
             data->temperature, data->humidity, data->pressure,
             data->gas_resistance, data->gas_valid ? "valid" : "warming");

    return ESP_OK;
}
