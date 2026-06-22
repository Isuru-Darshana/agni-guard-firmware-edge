#include "bme680_bsec.h"
#include "bme68x_hal.h"
#include "bme68x.h"
#include "bsec_interface.h"
#include "bsec_iaq.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG      = "BME680";
static const char *NVS_NS   = "bsec";
static const char *NVS_KEY  = "state";

// ── Static state ──────────────────────────────────────────────
static struct bme68x_dev  bme680_dev;
static i2c_port_t         i2c_port = BME680_I2C_PORT;
static int64_t            next_call_ns = 0;

// ── BSEC outputs we want ──────────────────────────────────────
static bsec_sensor_configuration_t requested_outputs[] = {
    { BSEC_OUTPUT_IAQ,                   BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_CO2_EQUIVALENT,        BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_RAW_TEMPERATURE,       BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_RAW_PRESSURE,          BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_RAW_HUMIDITY,          BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_RAW_GAS,               BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_STABILIZATION_STATUS,  BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_RUN_IN_STATUS,         BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
                                         BSEC_SAMPLE_RATE },
    { BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
                                         BSEC_SAMPLE_RATE },
};
#define N_OUTPUTS (sizeof(requested_outputs) / \
                   sizeof(requested_outputs[0]))

// ── Init ─────────────────────────────────────────────────────
esp_err_t bme680_bsec_init(void) {
    // ── I2C bus init (shared with BME280 on I2C_NUM_0) ───────
    // Only init if not already done by bme280_init()
    // i2c_driver_install returns ESP_ERR_INVALID_STATE if
    // already installed — that's fine, just continue
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BME680_SDA_PIN,
        .scl_io_num       = BME680_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(BME680_I2C_PORT, &conf);
    esp_err_t i2c_ret = i2c_driver_install(BME680_I2C_PORT,
                                            I2C_MODE_MASTER,
                                            0, 0, 0);
    if (i2c_ret != ESP_OK &&
        i2c_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C init failed: %s",
                 esp_err_to_name(i2c_ret));
        return i2c_ret;
    }

    // ── BME68x device init ────────────────────────────────────
    bme680_dev.intf     = BME68X_I2C_INTF;
    bme680_dev.read     = bme68x_i2c_read;
    bme680_dev.write    = bme68x_i2c_write;
    bme680_dev.delay_us = bme68x_delay_us;
    bme680_dev.intf_ptr = &i2c_port;
    bme680_dev.amb_temp = 25;  // initial ambient temp estimate

    int8_t rslt = bme68x_init(&bme680_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "BME68x init failed: %d", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BME680 chip found (variant=%d)",
             (int)bme680_dev.variant_id);

    // ── BSEC2 library init ────────────────────────────────────
    bsec_library_return_t bret = bsec_init();
    if (bret != BSEC_OK) {
        ESP_LOGE(TAG, "BSEC init failed: %d", bret);
        return ESP_FAIL;
    }

    // Log BSEC version
    bsec_version_t ver;
    bsec_get_version(&ver);
    ESP_LOGI(TAG, "BSEC v%d.%d.%d.%d",
             ver.major, ver.minor,
             ver.major_bugfix, ver.minor_bugfix);

    // ── Load config for BME680 3.3V 3s 4d ────────────────────
    // bsec_iaq.c/h from algo/bsec_IAQ/config/bme680/
    //   bme680_iaq_33v_3s_4d/
    bret = bsec_set_configuration(bsec_config_iaq,
                                   sizeof(bsec_config_iaq),
                                   NULL, 0);
    if (bret != BSEC_OK) {
        ESP_LOGE(TAG, "BSEC config failed: %d", bret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BSEC config loaded (BME680 3.3V 3s 4d)");

    // ── Try loading saved calibration state from NVS ──────────
    bme680_bsec_load_state();

    // ── Subscribe to outputs ──────────────────────────────────
    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required = BSEC_MAX_PHYSICAL_SENSOR;

    bret = bsec_update_subscription(requested_outputs,
                                     N_OUTPUTS,
                                     required_sensor_settings,
                                     &n_required);
    if (bret != BSEC_OK) {
        ESP_LOGE(TAG, "BSEC subscription failed: %d", bret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BSEC init OK — %d sensor configs required",
             n_required);

    // Set first call time to now
    next_call_ns = esp_timer_get_time() * 1000LL;

    return ESP_OK;
}

// ── Read ─────────────────────────────────────────────────────
esp_err_t bme680_bsec_read(bme680_data_t *data) {
    memset(data, 0, sizeof(bme680_data_t));

    int64_t now_ns = esp_timer_get_time() * 1000LL;
    data->timestamp_ns = now_ns;

    // ── Get sensor settings from BSEC ────────────────────────
    bsec_bme_settings_t sensor_settings;
    bsec_library_return_t bret = bsec_sensor_control(
        now_ns, &sensor_settings);

    if (bret != BSEC_OK) {
        ESP_LOGW(TAG, "bsec_sensor_control: %d", bret);
        return ESP_FAIL;
    }

    // Store next call time
    next_call_ns = sensor_settings.next_call;

    // Check if measurement is needed now
    if (sensor_settings.trigger_measurement == 0) {
        ESP_LOGD(TAG, "No measurement needed yet");
        return ESP_ERR_NOT_FINISHED;
    }

    // ── Configure BME680 per BSEC instructions ────────────────
    struct bme68x_conf conf;
    conf.filter  = BME68X_FILTER_OFF;
    conf.odr     = BME68X_ODR_NONE;
    conf.os_hum  = sensor_settings.humidity_oversampling;
    conf.os_pres = sensor_settings.pressure_oversampling;
    conf.os_temp = sensor_settings.temperature_oversampling;
    bme68x_set_conf(&conf, &bme680_dev);

    // ── Configure heater ──────────────────────────────────────
    struct bme68x_heatr_conf heatr_conf;
    heatr_conf.enable     = sensor_settings.run_gas;
    heatr_conf.heatr_temp = sensor_settings.heater_temperature;
    heatr_conf.heatr_dur  = sensor_settings.heater_duration;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE,
                           &heatr_conf, &bme680_dev);

    // ── Trigger forced mode measurement ──────────────────────
    bme68x_set_op_mode(BME68X_FORCED_MODE, &bme680_dev);

    // ── Wait for measurement ──────────────────────────────────
    uint32_t delay_us = bme68x_get_meas_dur(
        BME68X_FORCED_MODE, &conf, &bme680_dev) +
        (heatr_conf.heatr_dur * 1000);
    bme680_dev.delay_us(delay_us, bme680_dev.intf_ptr);

    // ── Read raw data ─────────────────────────────────────────
    struct bme68x_data raw_data;
    uint8_t n_data = 0;
    int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE,
                                   &raw_data, &n_data,
                                   &bme680_dev);
    if (rslt != BME68X_OK || n_data == 0) {
        ESP_LOGW(TAG, "BME68x read failed: %d", rslt);
        return ESP_FAIL;
    }

    // ── Feed raw data into BSEC ───────────────────────────────
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_inputs = 0;

    if (sensor_settings.process_data & BSEC_PROCESS_TEMPERATURE) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_TEMPERATURE;
        inputs[n_inputs].signal     = raw_data.temperature;
        inputs[n_inputs].time_stamp = now_ns;
        n_inputs++;
    }
    if (sensor_settings.process_data & BSEC_PROCESS_HUMIDITY) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_HUMIDITY;
        inputs[n_inputs].signal     = raw_data.humidity;
        inputs[n_inputs].time_stamp = now_ns;
        n_inputs++;
    }
    if (sensor_settings.process_data & BSEC_PROCESS_PRESSURE) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_PRESSURE;
        inputs[n_inputs].signal     = raw_data.pressure;
        inputs[n_inputs].time_stamp = now_ns;
        n_inputs++;
    }
    if (sensor_settings.process_data & BSEC_PROCESS_GAS) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_GASRESISTOR;
        inputs[n_inputs].signal     = raw_data.gas_resistance;
        inputs[n_inputs].time_stamp = now_ns;
        n_inputs++;

        // Also pass heat source flag
        inputs[n_inputs].sensor_id  = BSEC_INPUT_HEATSOURCE;
        inputs[n_inputs].signal     = 0;
        inputs[n_inputs].time_stamp = now_ns;
        n_inputs++;
    }

    // ── Process with BSEC ────────────────────────────────────
    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t n_outputs = BSEC_NUMBER_OUTPUTS;

    bret = bsec_do_steps(inputs, n_inputs,
                          outputs, &n_outputs);
    if (bret != BSEC_OK) {
        ESP_LOGW(TAG, "bsec_do_steps: %d", bret);
        return ESP_FAIL;
    }

    // ── Extract outputs ───────────────────────────────────────
    for (uint8_t i = 0; i < n_outputs; i++) {
        switch (outputs[i].sensor_id) {

        case BSEC_OUTPUT_IAQ:
            data->iaq          = outputs[i].signal;
            data->iaq_accuracy = outputs[i].accuracy;
            break;

        case BSEC_OUTPUT_CO2_EQUIVALENT:
            data->co2_equivalent = outputs[i].signal;
            break;

        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            data->voc_equivalent = outputs[i].signal;
            break;

        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            data->temperature = outputs[i].signal;
            break;

        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            data->humidity = outputs[i].signal;
            break;

        case BSEC_OUTPUT_RAW_PRESSURE:
            data->pressure = outputs[i].signal / 100.0f; // Pa→hPa
            break;

        case BSEC_OUTPUT_RAW_GAS:
            // Raw gas resistance in Ohms → convert to kΩ
            data->gas_resistance = outputs[i].signal / 1000.0f;
            data->gas_valid      = (outputs[i].accuracy != 0);
            break;

        default:
            break;
        }
    }

    ESP_LOGI(TAG,
             "T=%.2f°C H=%.2f%% P=%.2fhPa "
             "Gas=%.2fkΩ IAQ=%.1f(%d) "
             "CO2=%.1fppm VOC=%.2fppm",
             data->temperature, data->humidity,
             data->pressure, data->gas_resistance,
             data->iaq, data->iaq_accuracy,
             data->co2_equivalent, data->voc_equivalent);

    return ESP_OK;
}

// ── Save BSEC state to NVS ────────────────────────────────────
// Call before deep sleep to preserve calibration
esp_err_t bme680_bsec_save_state(void) {
    uint8_t  state_buf[BSEC_MAX_STATE_BLOB_SIZE];
    uint32_t state_len = 0;

    bsec_library_return_t bret = bsec_get_state(
        0, state_buf, sizeof(state_buf),
        NULL, 0, &state_len);

    if (bret != BSEC_OK || state_len == 0) {
        ESP_LOGW(TAG, "bsec_get_state failed: %d", bret);
        return ESP_FAIL;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(handle, NVS_KEY, state_buf, state_len);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "BSEC state saved (%lu bytes)",
             (unsigned long)state_len);
    return ret;
}

// ── Load BSEC state from NVS ──────────────────────────────────
esp_err_t bme680_bsec_load_state(void) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved BSEC state — starting fresh");
        return ESP_ERR_NOT_FOUND;
    }
    if (ret != ESP_OK) return ret;

    uint8_t  state_buf[BSEC_MAX_STATE_BLOB_SIZE];
    size_t   state_len = sizeof(state_buf);

    ret = nvs_get_blob(handle, NVS_KEY, state_buf, &state_len);
    nvs_close(handle);

    if (ret != ESP_OK) return ret;

    bsec_library_return_t bret = bsec_set_state(
        state_buf, state_len, NULL, 0);

    if (bret != BSEC_OK) {
        ESP_LOGW(TAG, "bsec_set_state failed: %d", bret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BSEC state restored (%d bytes)",
             (int)state_len);
    return ESP_OK;
}

// ── Get BSEC version ─────────────────────────────────────────
void bme680_bsec_get_version(uint8_t *major, uint8_t *minor,
                               uint8_t *major_bugfix,
                               uint8_t *minor_bugfix) {
    bsec_version_t ver;
    bsec_get_version(&ver);
    *major       = ver.major;
    *minor       = ver.minor;
    *major_bugfix = ver.major_bugfix;
    *minor_bugfix = ver.minor_bugfix;
}