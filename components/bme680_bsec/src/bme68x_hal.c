#include "bme68x_hal.h"
#include "bme680_bsec.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BME68x_HAL";

// ── I2C write — called by BME68x API ─────────────────────────
// reg_addr: register to write to
// reg_data: data bytes to write
// length:   number of bytes
// intf_ptr: pointer to i2c port (cast from void*)
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr,
                                        const uint8_t *reg_data,
                                        uint32_t length,
                                        void *intf_ptr) {
    i2c_port_t port = *((i2c_port_t *)intf_ptr);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (BME680_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, length, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// ── I2C read — called by BME68x API ──────────────────────────
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr,
                                       uint8_t *reg_data,
                                       uint32_t length,
                                       void *intf_ptr) {
    i2c_port_t port = *((i2c_port_t *)intf_ptr);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (BME680_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (BME680_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, reg_data, length - 1,
                        I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + length - 1,
                         I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// ── Delay — called by BME68x API ─────────────────────────────
// period: microseconds
void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    if (period < 1000) {
        // Short delays: busy wait
        esp_rom_delay_us(period);
    } else {
        // Longer delays: FreeRTOS tick
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    }
}