#include "nvs_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG        = "NVS";
static const char *NVS_NS     = "agni";
static const char *KEY_GAS    = "gas_baseline";
static const char *KEY_CALIB  = "calib_done";

esp_err_t nvs_config_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition issue — erasing and reinitialising");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_LOGI(TAG, "NVS init OK");
    return ret;
}

esp_err_t nvs_config_save_baseline(float gas_baseline) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    // Store float as uint32 via union
    union { float f; uint32_t u; } conv;
    conv.f = gas_baseline;
    ret = nvs_set_u32(handle, KEY_GAS, conv.u);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Baseline saved: %.2f kΩ", gas_baseline);
    return ret;
}

esp_err_t nvs_config_load_baseline(float *gas_baseline) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No baseline in NVS");
        *gas_baseline = 0.0f;
        return ESP_ERR_NOT_FOUND;
    }
    if (ret != ESP_OK) return ret;

    union { float f; uint32_t u; } conv;
    ret = nvs_get_u32(handle, KEY_GAS, &conv.u);
    nvs_close(handle);

    if (ret == ESP_OK) {
        *gas_baseline = conv.f;
        ESP_LOGI(TAG, "Baseline loaded: %.2f kΩ", *gas_baseline);
    }
    return ret;
}

esp_err_t nvs_config_save_calibration_flag(bool done) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u8(handle, KEY_CALIB, done ? 1 : 0);
    nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t nvs_config_load_calibration_flag(bool *done) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        *done = false;
        return ESP_ERR_NOT_FOUND;
    }
    if (ret != ESP_OK) return ret;

    uint8_t val = 0;
    ret = nvs_get_u8(handle, KEY_CALIB, &val);
    nvs_close(handle);
    *done = (val == 1);
    return ret;
}

void nvs_config_erase(void) {
    nvs_flash_erase();
    nvs_flash_init();
    ESP_LOGW(TAG, "NVS erased");
}