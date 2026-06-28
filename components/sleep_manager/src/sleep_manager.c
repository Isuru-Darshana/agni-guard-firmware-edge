#include "sleep_manager.h"
#include "power_manager.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SLEEP";

const uint8_t SLEEP_MINUTES[] = {1, 4, 3, 2, 60};
const char*   STAGE_NAMES[]   = {"NORMAL", "ALERT", "ELEVATED", "CRITICAL", "RAIN"};

// ============================================================
// RTC memory definitions — one place only
// ============================================================
RTC_DATA_ATTR int          g_boot_count       = 0;
RTC_DATA_ATTR bool         g_calibration_done = false;
RTC_DATA_ATTR float        g_gas_baseline     = 0.0f;
RTC_DATA_ATTR fire_stage_t g_current_stage    = STAGE_NORMAL;
RTC_DATA_ATTR uint16_t     g_sequence_number  = 0;
RTC_DATA_ATTR float        g_cal_sum          = 0.0f;
RTC_DATA_ATTR uint32_t     g_cal_count        = 0;

void sleep_manager_init(void) {
    power_manager_release_pins();

    g_boot_count++;
    g_sequence_number++;

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Boot #%d — Timer wakeup | Stage: %s | Seq: %d",
                 g_boot_count,
                 STAGE_NAMES[g_current_stage],
                 g_sequence_number);
    } else {
        ESP_LOGI(TAG, "Boot #%d — Power-on reset — clearing RTC state",
                 g_boot_count);
        g_calibration_done = false;
        g_gas_baseline     = 0.0f;
        g_current_stage    = STAGE_NORMAL;
        g_sequence_number  = 0;
        g_cal_sum          = 0.0f;
        g_cal_count        = 0;
    }
}

bool sleep_manager_is_timer_wakeup(void) {
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

fire_stage_t sleep_manager_get_stage(void) {
    return g_current_stage;
}

void sleep_manager_set_stage(fire_stage_t stage) {
    if (stage <= STAGE_RAIN) {
        g_current_stage = stage;
    }
}

void sleep_manager_enter_deep_sleep(fire_stage_t stage) {
    g_current_stage = stage;

    uint8_t  minutes  = SLEEP_MINUTES[stage];
    uint64_t sleep_us = (uint64_t)minutes * 60ULL * 1000000ULL;

    ESP_LOGI(TAG, "Deep sleep: %d min — stage: %s",
             minutes, STAGE_NAMES[stage]);

    fflush(stdout);
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);
    power_manager_hold_pins();
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
}

void sleep_manager_enter_deep_sleep_with_soc(fire_stage_t stage,
                                              uint8_t soc) {
    uint8_t  minutes;
    uint64_t sleep_us;

    // TODO: re-enable critical battery sleep when power management is stable
    // if (soc <= BATTERY_CRIT_SOC) {
    //     minutes = 30;
    //     ESP_LOGW(TAG, "Battery CRITICAL (%d%%) — forced 30min sleep", soc);
    // } else
    if (soc <= BATTERY_LOW_SOC) {
        uint8_t normal = SLEEP_MINUTES[stage];
        minutes = (normal < 10) ? 10 : normal;
        ESP_LOGW(TAG, "Battery LOW (%d%%) — extending sleep to %dmin",
                 soc, minutes);
    } else {
        g_current_stage = stage;
        sleep_manager_enter_deep_sleep(stage);
        return;
    }

    g_current_stage = stage;
    sleep_us = (uint64_t)minutes * 60ULL * 1000000ULL;

    ESP_LOGI(TAG, "Deep sleep: %dmin (stage=%s SOC=%d%%)",
             minutes, STAGE_NAMES[stage], soc);

    fflush(stdout);
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);
    power_manager_hold_pins();
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
}
