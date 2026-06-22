#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <stdint.h>

#define PMS7003_UART_PORT   UART_NUM_1
#define PMS7003_RX_PIN      GPIO_NUM_39
#define PMS7003_TX_PIN      UART_PIN_NO_CHANGE  // TX not needed
#define PMS7003_BAUD        9600
#define PMS7003_BUF_SIZE    256

typedef struct {
    uint16_t pm2_5;   // µg/m³ atmospheric
    uint16_t pm10;    // µg/m³ atmospheric
    uint16_t pm1_0;   // µg/m³ atmospheric
} pms7003_data_t;

esp_err_t pms7003_init(void);
esp_err_t pms7003_read(pms7003_data_t *data);