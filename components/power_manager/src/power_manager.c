#include "power_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"

static const char *TAG = "POWER";

esp_err_t power_manager_init(void) {
    // Configure sensor power pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_BME680_EN) |
                        (1ULL << PIN_BME280_EN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // SD_EN is active LOW — P-MOSFET
    gpio_config_t sd_conf = {
        .pin_bit_mask = (1ULL << PIN_SD_EN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&sd_conf);

    // Start with everything OFF
    gpio_set_level(PIN_BME680_EN, 0);
    gpio_set_level(PIN_BME280_EN, 0);
    gpio_set_level(PIN_SD_EN, 1);  // HIGH = OFF for P-MOSFET

    ESP_LOGI(TAG, "Power manager init OK — all sensors OFF");
    return ESP_OK;
}

void power_manager_sensors_on(void) {
    gpio_set_level(PIN_BME680_EN, 1);
    gpio_set_level(PIN_BME280_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(2000)); // PMS7003 needs 2s boot time
    ESP_LOGI(TAG, "Sensors ON");
}

void power_manager_sensors_off(void) {
    gpio_set_level(PIN_BME680_EN, 0);
    gpio_set_level(PIN_BME280_EN, 0);
    ESP_LOGI(TAG, "Sensors OFF");
}

void power_manager_sd_on(void) {
    gpio_set_level(PIN_SD_EN, 0);  // LOW = ON for P-MOSFET
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "SD card ON");
}

void power_manager_sd_off(void) {
    gpio_set_level(PIN_SD_EN, 1);  // HIGH = OFF for P-MOSFET
    ESP_LOGI(TAG, "SD card OFF");
}

void power_manager_hold_pins(void) {
    // Hold GPIO state during deep sleep
    // Only RTC-capable GPIOs (0-21) can use rtc_gpio_hold_en
    // PIN_BME680_EN = GPIO1  — RTC capable
    // PIN_BME280_EN = GPIO17 — RTC capable
    // PIN_SD_EN     = GPIO40 — NOT RTC capable, use gpio_hold_en
    rtc_gpio_hold_en(PIN_BME680_EN);
    rtc_gpio_hold_en(PIN_BME280_EN);
    gpio_hold_en(PIN_SD_EN);
    ESP_LOGI(TAG, "GPIO holds set for deep sleep");
}

void power_manager_release_pins(void) {
    // Release holds on wakeup — must be called before reconfiguring GPIO
    rtc_gpio_hold_dis(PIN_BME680_EN);
    rtc_gpio_hold_dis(PIN_BME280_EN);
    gpio_hold_dis(PIN_SD_EN);
    ESP_LOGI(TAG, "GPIO holds released");
}