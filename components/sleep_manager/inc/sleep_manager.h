#pragma once

#include "esp_sleep.h"
#include "esp_attr.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STAGE_NORMAL   = 0,
    STAGE_ALERT    = 1,
    STAGE_ELEVATED = 2,
    STAGE_CRITICAL = 3,
    STAGE_RAIN     = 4
} fire_stage_t;

// Battery thresholds
#define BATTERY_LOW_SOC   10   // % — extend sleep
#define BATTERY_CRIT_SOC   5   // % — maximum sleep

extern const uint8_t SLEEP_MINUTES[];
extern const char*   STAGE_NAMES[];

// RTC memory — defined once in sleep_manager.c
extern int           g_boot_count;
extern bool          g_calibration_done;
extern float         g_gas_baseline;
extern fire_stage_t  g_current_stage;
extern uint16_t      g_sequence_number;

void         sleep_manager_init(void);
void         sleep_manager_enter_deep_sleep(fire_stage_t stage);
void         sleep_manager_enter_deep_sleep_with_soc(
                 fire_stage_t stage, uint8_t soc);
bool         sleep_manager_is_timer_wakeup(void);
fire_stage_t sleep_manager_get_stage(void);
void         sleep_manager_set_stage(fire_stage_t stage);
