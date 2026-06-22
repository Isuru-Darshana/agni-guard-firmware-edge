#pragma once

#include "bme68x.h"
#include <stdint.h>

// HAL functions — implemented in bme68x_hal.c
// Called by BME68x API internally via function pointers
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr,
                                        const uint8_t *reg_data,
                                        uint32_t length,
                                        void *intf_ptr);

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr,
                                       uint8_t *reg_data,
                                       uint32_t length,
                                       void *intf_ptr);

void bme68x_delay_us(uint32_t period, void *intf_ptr);