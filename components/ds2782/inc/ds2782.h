#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ── I2C Address ──────────────────────────────────────────────
// Factory default: 0110100b = 0x34
// Stored in EEPROM register 0x7E — changeable
#define DS2782_ADDR     0x34

// ── Register Map (Table 2, page 23) ──────────────────────────
#define DS2782_REG_STATUS    0x01  // Status Register R/W
#define DS2782_REG_RAAC_MSB  0x02  // Remaining Active Absolute Capacity MSB R
#define DS2782_REG_RAAC_LSB  0x03  // Remaining Active Absolute Capacity LSB R
#define DS2782_REG_RSAC_MSB  0x04  // Remaining Standby Absolute Capacity MSB R
#define DS2782_REG_RSAC_LSB  0x05  // Remaining Standby Absolute Capacity LSB R
#define DS2782_REG_RARC      0x06  // Remaining Active Relative Capacity % R
#define DS2782_REG_RSRC      0x07  // Remaining Standby Relative Capacity % R
#define DS2782_REG_IAVG_MSB  0x08  // Average Current MSB R
#define DS2782_REG_IAVG_LSB  0x09  // Average Current LSB R
#define DS2782_REG_TEMP_MSB  0x0A  // Temperature MSB R
#define DS2782_REG_TEMP_LSB  0x0B  // Temperature LSB R
#define DS2782_REG_VOLT_MSB  0x0C  // Voltage MSB R
#define DS2782_REG_VOLT_LSB  0x0D  // Voltage LSB R
#define DS2782_REG_CURR_MSB  0x0E  // Current MSB R
#define DS2782_REG_CURR_LSB  0x0F  // Current LSB R
#define DS2782_REG_ACR_MSB   0x10  // Accumulated Current MSB R/W
#define DS2782_REG_ACR_LSB   0x11  // Accumulated Current LSB R/W
#define DS2782_REG_ACRL_MSB  0x12  // Low Accumulated Current MSB R
#define DS2782_REG_ACRL_LSB  0x13  // Low Accumulated Current LSB R
#define DS2782_REG_AS        0x14  // Age Scalar R/W
#define DS2782_REG_SFR       0x15  // Special Feature Register R/W
#define DS2782_REG_FULL_MSB  0x16  // Full Capacity MSB R
#define DS2782_REG_FULL_LSB  0x17  // Full Capacity LSB R
#define DS2782_REG_AE_MSB    0x18  // Active Empty MSB R
#define DS2782_REG_AE_LSB    0x19  // Active Empty LSB R
#define DS2782_REG_SE_MSB    0x1A  // Standby Empty MSB R
#define DS2782_REG_SE_LSB    0x1B  // Standby Empty LSB R
#define DS2782_REG_EEPROM    0x1F  // EEPROM Register R/W

// ── Parameter EEPROM Block 1 (Table 3, page 24) ──────────────
#define DS2782_REG_CONTROL   0x60  // Control Register
#define DS2782_REG_AB        0x61  // Accumulation Bias
#define DS2782_REG_AC_MSB    0x62  // Aging Capacity MSB
#define DS2782_REG_AC_LSB    0x63  // Aging Capacity LSB
#define DS2782_REG_VCHG      0x64  // Charge Voltage — units: 19.52mV/LSB
#define DS2782_REG_IMIN      0x65  // Minimum Charge Current — units: 50µV/LSB
#define DS2782_REG_VAE       0x66  // Active Empty Voltage — units: 19.52mV/LSB
#define DS2782_REG_IAE       0x67  // Active Empty Current — units: 200µV/LSB
#define DS2782_REG_AE40      0x68  // Active Empty at 40°C
#define DS2782_REG_RSNSP     0x69  // Sense Resistor Prime — units: mhos (S)
#define DS2782_REG_FULL40_H  0x6A  // Full Capacity at 40°C MSB
#define DS2782_REG_FULL40_L  0x6B  // Full Capacity at 40°C LSB
#define DS2782_REG_RSGAIN_H  0x78  // Sense Resistor Gain MSB
#define DS2782_REG_RSGAIN_L  0x79  // Sense Resistor Gain LSB
#define DS2782_REG_SLAVE     0x7E  // Programmable Slave Address

// ── Function Command Register (page 26) ──────────────────────
#define DS2782_REG_FUNC_CMD  0xFE  // Function Command Register W

// ── Function Commands (Table 5, page 27) ─────────────────────
#define DS2782_CMD_COPY_BLOCK0    0x42  // Copy shadow RAM → EEPROM block 0
#define DS2782_CMD_COPY_BLOCK1    0x44  // Copy shadow RAM → EEPROM block 1
#define DS2782_CMD_RECALL_BLOCK0  0xB2  // Recall EEPROM block 0 → shadow RAM
#define DS2782_CMD_RECALL_BLOCK1  0xB4  // Recall EEPROM block 1 → shadow RAM
#define DS2782_CMD_LOCK_BLOCK0    0x63  // Lock block 0 permanently
#define DS2782_CMD_LOCK_BLOCK1    0x66  // Lock block 1 permanently

// ── STATUS Register Bits (page 19) ───────────────────────────
#define DS2782_STATUS_CHGTF   0x80  // Charge termination flag (read only)
#define DS2782_STATUS_AEF     0x40  // Active empty flag (read only)
#define DS2782_STATUS_SEF     0x20  // Standby empty flag (read only)
#define DS2782_STATUS_LEARNF  0x10  // Learning in progress (read only)
#define DS2782_STATUS_UVF     0x04  // Undervoltage flag (R/W)
#define DS2782_STATUS_PORF    0x02  // Power-on reset flag (R/W) — clear on boot

// ── Measurement LSBs at Rsns = 20mΩ (from datasheet tables) ──
// Voltage:      4.88mV per LSB, 11-bit left-aligned (shift >>5)
// Temperature:  0.125°C per LSB, 11-bit left-aligned (shift >>5)
// Current:      1.5625µV / 0.020Ω = 78.125µA = 0.078125mA per LSB, 16-bit signed
// IAVG:         same units as current, 16-bit signed, updated every 28s
// RAAC:         1.6mAh per LSB (Figure 14, page 17)
// RARC:         1% per LSB, direct 0-100 (Figure 16, page 18)
#define DS2782_VOLT_LSB_V     0.00488f    // V per LSB
#define DS2782_TEMP_LSB_C     0.125f      // °C per LSB
#define DS2782_CURR_LSB_MA    0.078125f   // mA per LSB at 20mΩ
#define DS2782_RAAC_LSB_MAH   1.6f        // mAh per LSB

// ── EEPROM values for Samsung 30Q at Rsns=20mΩ ───────────────
// RSNSP = 1/0.020 = 50 = 0x32
// VCHG  = 4150mV / 19.52mV = 212 = 0xD4
// VAE   = 3000mV / 19.52mV = 153 = 0x99
// IMIN  = 100mA / (50µV/0.020Ω) = 100/2.5 = 40 = 0x28
// IAE   = 500mA / (200µV/0.020Ω) = 500/10 = 50 = 0x32
// AC    = 2500mAh / 0.3125mAh = 8000 = 0x1F40
#define DS2782_EEPROM_RSNSP   0x32
#define DS2782_EEPROM_VCHG    0xD4
#define DS2782_EEPROM_VAE     0x99
#define DS2782_EEPROM_IMIN    0x28
#define DS2782_EEPROM_IAE     0x32
#define DS2782_EEPROM_AC_MSB  0x1F
#define DS2782_EEPROM_AC_LSB  0x40

// ── Data struct ───────────────────────────────────────────────
typedef struct {
    float    voltage;        // V  (from 0x0C-0x0D)
    float    current;        // mA (from 0x0E-0x0F instantaneous)
    float    avg_current;    // mA (from 0x08-0x09 28s average)
    float    temperature;    // °C (from 0x0A-0x0B)
    uint8_t  soc_percent;    // %  (from 0x06 RARC direct 0-100)
    uint16_t capacity_mah;   // mAh (from 0x02-0x03 RAAC)
    bool     learning_done;  // true = LEARNF cleared after CHGTF
    bool     porf_was_set;   // true = PORF was set on this boot
} ds2782_data_t;

// ── API ───────────────────────────────────────────────────────
esp_err_t ds2782_init(i2c_port_t port, uint8_t addr);
esp_err_t ds2782_read(i2c_port_t port, uint8_t addr,
                      ds2782_data_t *data);
uint8_t   ds2782_get_soc_safe(i2c_port_t port, uint8_t addr,
                               float *voltage_out);
void      ds2782_raw_dump(i2c_port_t port, uint8_t addr);