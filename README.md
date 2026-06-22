# AGNI GUARD — Edge Node v2.0

ESP32-S3 firmware. Wakes from deep sleep, reads sensors, transmits via LoRa, waits ACK, sleeps again.

---

## Directory Structure

```
edge_node/
├── main/
│   ├── edge_node.c          — app_main, boot sequence (14 steps)
│   ├── sleep_manager.c/h    — RTC state, deep sleep, battery protection
│   ├── lora_protocol.c/h    — LoRa packet TX and ACK parsing
│   └── calibration.c/h      — BME680 gas baseline calibration
└── components/
    ├── power_manager/       — GPIO power rails for sensors + SD card
    ├── sx1278/              — SX1278 LoRa radio SPI driver
    ├── bme680_bsec/         — BME680 + Bosch BSEC2 library (IAQ, CO2eq, VOCeq)
    ├── bme280/              — BME280 temp/humidity/pressure (I2C)
    ├── pms7003/             — PMS7003 particulate matter (UART)
    ├── ds2782/              — DS2782 battery fuel gauge (I2C, 2 cells)
    ├── nvs_config/          — NVS flash: gas baseline + calibration flag
    └── sd_logger/           — SD card CSV logging
```

---

## Boot Sequence (`edge_node.c`)

| Step | Action |
|------|--------|
| 1 | NVS flash init |
| 2 | Sleep manager init — releases GPIO holds, increments boot/seq counters |
| 3 | Power manager init → sensors ON (2 s delay for PMS7003) |
| 4 | DS2782 battery measurement (2 cells via I2C) |
| 5 | Sensor init: BME280, BME680/BSEC, PMS7003 |
| 6 | SX1278 LoRa init |
| 7 | Load gas baseline from NVS into RTC memory |
| 8 | Read all sensors |
| 9 | Update gas baseline (running avg: 90% old + 10% new) |
| 10 | SD card ON → log entry → SD card OFF |
| 11 | LoRa transmit → wait ACK (30 s timeout) |
| 12 | Save BSEC calibration state + gas baseline to NVS |
| 13 | Power off sensors + LoRa sleep |
| 14 | Deep sleep (duration based on fire stage + battery SOC) |

---

## Fire Stages

Received in ACK from gateway. Controls sleep interval.

| Stage | Value | Sleep interval |
|-------|-------|----------------|
| `STAGE_NORMAL` | 0 | 1 min |
| `STAGE_ALERT` | 1 | 4 min |
| `STAGE_ELEVATED` | 2 | 3 min |
| `STAGE_CRITICAL` | 3 | 2 min |
| `STAGE_RAIN` | 4 | 60 min |

---

## Battery Protection (`sleep_manager.c`)

Overrides fire-stage sleep time when battery is low.

| Condition | SOC threshold | Sleep duration |
|-----------|---------------|----------------|
| Normal | > 10% | Stage table above |
| Low battery | ≤ 10% | max(stage\_minutes, 10 min) |
| Critical battery | ≤ 5% | 30 min (fixed) |

---

## RTC Memory (survives deep sleep)

Defined in `sleep_manager.c`, declared `extern` in `sleep_manager.h`.

| Variable | Type | Purpose |
|----------|------|---------|
| `g_boot_count` | `int` | Incremented every wake |
| `g_sequence_number` | `uint16_t` | Packet sequence counter |
| `g_current_stage` | `fire_stage_t` | Last stage received from gateway |
| `g_gas_baseline` | `float` | Running average gas resistance (kΩ) |
| `g_calibration_done` | `bool` | First-boot calibration flag |

On power-on reset (not timer wakeup): all fields reset to defaults.

---

## LoRa Protocol (`lora_protocol.c`)

**Radio config (SX1278):** 433 MHz · SF10 · BW 125 kHz · CR 4/5 · CRC on · +17 dBm · sync word `0x12`

### TX packet (CSV)

```
D,<nodeID>,<T680>,<H680>,<P680>,<Gas>,<T280>,<H280>,<P280>,<PM2.5>,<PM10>,<SOC>,<Voltage>,<seq>,<baseline>
```

| Field | Source | Unit |
|-------|--------|------|
| T680/H680/P680 | BME680 BSEC heat-compensated | °C / % / hPa |
| Gas | BME680 raw gas resistance | kΩ |
| T280/H280/P280 | BME280 | °C / % / hPa |
| PM2.5 / PM10 | PMS7003 | µg/m³ |
| SOC | DS2782 min(cell1, cell2) | % |
| Voltage | DS2782 cell1 + cell2 | V |
| seq | `g_sequence_number` | — |
| baseline | `g_gas_baseline` | kΩ |

### ACK packet (CSV)

```
A,<nodeID>,<fireStage>,<seq>
```

Gateway echoes node ID and sequence, sets next fire stage. Timeout: 30 s.

---

## Pin Assignments

### Power Rails

| Pin | GPIO | Logic | Controls |
|-----|------|-------|----------|
| `PIN_BME680_EN` | GPIO1 | Active HIGH | BME680 power |
| `PIN_BME280_EN` | GPIO17 | Active HIGH | BME280 power |
| `PIN_SD_EN` | GPIO40 | Active LOW (P-MOSFET) | SD card power |

GPIO1 and GPIO17 are RTC-capable → use `rtc_gpio_hold_en` during deep sleep.
GPIO40 is not RTC-capable → uses `gpio_hold_en`.

### I2C Buses

| Bus | SDA | SCL | Devices |
|-----|-----|-----|---------|
| `I2C_NUM_0` | GPIO8 | GPIO9 | BME280 (0x76), BME680 (0x77), DS2782 Cell2 (0x34) |
| `I2C_NUM_1` | GPIO41 | GPIO42 | DS2782 Cell1 (0x34) |

### SX1278 SPI (defined in `sx1278.h`)

| Signal | GPIO |
|--------|------|
| MOSI | `SX1278_EDGE_MOSI` |
| MISO | `SX1278_EDGE_MISO` |
| SCK | `SX1278_EDGE_SCK` |
| NSS | `SX1278_EDGE_NSS` |
| RST | `SX1278_EDGE_RST` |
| DIO0 | `SX1278_EDGE_DIO0` |

---

## Sensors

### BME680 + BSEC2 (`components/bme680_bsec`)

Uses Bosch BSEC2 library (ultra-low-power profile, 3.3 V, 3 s, 4-day). Outputs:

- Temperature (heat-compensated), Humidity, Pressure
- IAQ score (0–500) + accuracy (0–3: UNRELIABLE / LOW / MEDIUM / HIGH)
- CO₂ equivalent (ppm), VOC equivalent (ppm)
- Raw gas resistance (kΩ)

BSEC calibration state persisted to NVS namespace `bsec` key `state` before each deep sleep.

### BME280 (`components/bme280`)

Temperature, humidity, pressure on I2C_NUM_0 address 0x76.

### PMS7003 (`components/pms7003`)

Laser particle counter. Outputs PM1.0, PM2.5, PM10 (µg/m³) via UART. Requires 2 s boot time after power-on.

### DS2782 (`components/ds2782`)

1-cell Li-Ion fuel gauge. Two instances: Cell1 on I2C_NUM_1, Cell2 on I2C_NUM_0. Both at address 0x34.
- Pack voltage = cell1_v + cell2_v
- Pack SOC = min(cell1_soc, cell2_soc)

---

## NVS Storage (`components/nvs_config`)

Namespace: `agni`

| Key | Type | Content |
|-----|------|---------|
| `gas_baseline` | `u32` (float union) | BME680 gas resistance baseline (kΩ) |
| `calib_done` | `u8` | Calibration complete flag |

Namespace: `bsec`

| Key | Type | Content |
|-----|------|---------|
| `state` | blob | BSEC2 calibration state |

---

## Calibration (`main/calibration.c`)

On first boot (no stored baseline): runs 60-minute calibration.
- Reads BME280 + PMS7003 every `CALIBRATION_INTERVAL_MS`
- Averages gas resistance over all readings
- Saves result to NVS + RTC memory
- Prints progress every 5 minutes

On subsequent boots: optionally recalibrate via UART prompt (30 s timeout, default = No).

---

## Build

ESP-IDF project targeting ESP32-S3.

```sh
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Node ID is hardcoded: `#define NODE_ID 1` in `main/edge_node.c`.
