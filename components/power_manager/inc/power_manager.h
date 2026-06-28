#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

// ============================================================
// Pin definitions — matches your PCB v2.0
// ============================================================
#define PIN_BME680_EN   GPIO_NUM_1   // Active HIGH
#define PIN_BME280_EN   GPIO_NUM_17  // Active HIGH
#define PIN_SD_EN       GPIO_NUM_40  // Active LOW (P-MOSFET)
// GPIO20 = USB D+ — cannot use as GPIO output while USB-Serial/JTAG active
// #define PIN_PMS_EN   GPIO_NUM_20  // conflicts with USB D+

// ============================================================
// API
// ============================================================
esp_err_t power_manager_init(void);
void      power_manager_sensors_on(void);
void      power_manager_sensors_off(void);
void      power_manager_sd_on(void);
void      power_manager_sd_off(void);
void      power_manager_hold_pins(void);    // call before deep sleep
void      power_manager_release_pins(void); // call on wakeup