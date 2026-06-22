#include "pms7003.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "PMS7003";

// ── Init ───────────────────────────────────────────────────
esp_err_t pms7003_init(void) {
    uart_config_t cfg = {
        .baud_rate  = PMS7003_BAUD,       // 9600 per datasheet
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE, // None per datasheet
        .stop_bits  = UART_STOP_BITS_1,    // 1 per datasheet
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(PMS7003_UART_PORT, &cfg);
    uart_set_pin(PMS7003_UART_PORT,
                 UART_PIN_NO_CHANGE,  // TX not needed (no commands sent)
                 PMS7003_RX_PIN,      // GPIO39 ← sensor Pin9 TX
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(PMS7003_UART_PORT,
                        PMS7003_BUF_SIZE * 2,  // RX buffer
                        0,                      // TX buffer (not needed)
                        0, NULL, 0);

    ESP_LOGI(TAG, "PMS7003 UART init OK — GPIO%d @ 9600 baud (active mode)",
             PMS7003_RX_PIN);
    return ESP_OK;
}

// ── Read ───────────────────────────────────────────────────
// PMS7003 active mode: sends 32-byte packet automatically
// Stable interval: 2.3s  Fast interval: 200-800ms
esp_err_t pms7003_read(pms7003_data_t *data) {
    const int MAX_RETRIES = 3;

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {

        // Flush stale data from RX buffer
        uart_flush(PMS7003_UART_PORT);
        vTaskDelay(pdMS_TO_TICKS(100));

        // ── Step 1: Find start bytes 0x42 0x4D ──────────────
        bool found = false;
        uint8_t byte;
        uint32_t start = xTaskGetTickCount();

        while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(5000)) {
            if (uart_read_bytes(PMS7003_UART_PORT, &byte, 1,
                                pdMS_TO_TICKS(200)) == 1) {
                if (byte == 0x42) {
                    uint8_t next;
                    if (uart_read_bytes(PMS7003_UART_PORT, &next, 1,
                                        pdMS_TO_TICKS(100)) == 1 &&
                        next == 0x4D) {
                        found = true;
                        break;
                    }
                }
            }
        }

        if (!found) {
            ESP_LOGW(TAG, "Start bytes not found (attempt %d/%d)",
                     attempt, MAX_RETRIES);
            continue;
        }

        // ── Step 2: Read remaining 30 bytes ─────────────────
        // Full packet is 32 bytes. We already consumed 0x42 0x4D.
        // Read the remaining 30 bytes into buf[2..31]
        uint8_t buf[32];
        buf[0] = 0x42;
        buf[1] = 0x4D;

        int read = uart_read_bytes(PMS7003_UART_PORT, &buf[2], 30,
                                   pdMS_TO_TICKS(2000));
        if (read != 30) {
            ESP_LOGW(TAG, "Incomplete packet: got %d/30 bytes (attempt %d/%d)",
                     read, attempt, MAX_RETRIES);
            continue;
        }

        // ── Step 3: Verify checksum ──────────────────────────
        // Checksum = sum of all bytes EXCEPT last 2
        // Per datasheet: "Start char1 + Start char2 + ... + data13"
        uint16_t calc_sum = 0;
        for (int i = 0; i < 30; i++) {
            calc_sum += buf[i];
        }
        uint16_t recv_sum = ((uint16_t)buf[30] << 8) | buf[31];

        if (calc_sum != recv_sum) {
            ESP_LOGW(TAG, "Checksum error: calc=0x%04X recv=0x%04X (attempt %d/%d)",
                     calc_sum, recv_sum, attempt, MAX_RETRIES);
            continue;
        }

        // ── Step 4: Verify frame length ──────────────────────
        // Datasheet: frame length = 2×13+2 = 28 (0x001C)
        uint16_t frame_len = ((uint16_t)buf[2] << 8) | buf[3];
        if (frame_len != 28) {
            ESP_LOGW(TAG, "Unexpected frame length: %d (attempt %d/%d)",
                     frame_len, attempt, MAX_RETRIES);
            // Don't reject — some firmware versions differ
        }

        // ── Step 5: Extract values ───────────────────────────
        // Using ATMOSPHERIC values (Data4-6) for outdoor deployment
        // Per datasheet note: CF=1 is for factory environment
        // Atmospheric values are correct for field deployment
        data->pm1_0 = ((uint16_t)buf[10] << 8) | buf[11]; // Data4 PM1.0 atm
        data->pm2_5 = ((uint16_t)buf[12] << 8) | buf[13]; // Data5 PM2.5 atm
        data->pm10  = ((uint16_t)buf[14] << 8) | buf[15]; // Data6 PM10  atm

        // Sanity check per datasheet effective range 0~500 µg/m³
        // Maximum range ≥1000 µg/m³ — anything above 2000 is suspect
        if (data->pm2_5 > 2000 || data->pm10 > 2000) {
            ESP_LOGW(TAG, "Unreasonable values PM2.5=%d PM10=%d (attempt %d/%d)",
                     data->pm2_5, data->pm10, attempt, MAX_RETRIES);
            if (attempt < MAX_RETRIES) continue;
        }

        if (attempt > 1) {
            ESP_LOGI(TAG, "Success on attempt %d", attempt);
        }

        ESP_LOGI(TAG, "PM1.0=%d  PM2.5=%d  PM10=%d  µg/m³ (atmospheric)",
                 data->pm1_0, data->pm2_5, data->pm10);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed after %d attempts", MAX_RETRIES);
    data->pm1_0 = 0;
    data->pm2_5 = 0;
    data->pm10  = 0;
    return ESP_FAIL;
}