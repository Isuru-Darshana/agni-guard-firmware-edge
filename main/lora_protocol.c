#include "lora_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "LORA_PROTO";

// CSV format:
// D,nodeID,T680,H680,P680,Gas,T280,H280,P280,
//   PM2.5,PM10,SOC,Voltage,seq,baseline
esp_err_t lora_transmit(sx1278_t *dev,
                        const lora_packet_t *pkt,
                        uint8_t node_id) {
    char buf[140];
    int len = snprintf(buf, sizeof(buf),
        "D,%d,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%d,%d,"
        "%d,%.2f,"
        "%d,%.2f",
        node_id,
        pkt->bme680_temp,
        pkt->bme680_humidity,
        pkt->bme680_pressure,
        pkt->bme680_gas,
        pkt->bme280_temp,
        pkt->bme280_humidity,
        pkt->bme280_pressure,
        pkt->pm2_5,
        pkt->pm10,
        pkt->battery_soc,
        pkt->battery_voltage,
        pkt->sequence,
        pkt->gas_baseline
    );

    if (len <= 0 || len >= (int)sizeof(buf)) {
        ESP_LOGE(TAG, "Packet format error");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TX [%d bytes]: %s", len, buf);

    esp_err_t ret = sx1278_send(dev,
                                (const uint8_t *)buf,
                                (uint8_t)len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TX OK");
    } else {
        ESP_LOGE(TAG, "TX failed: %s",
                 esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t lora_wait_ack(sx1278_t *dev,
                        uint8_t node_id,
                        fire_stage_t *stage_out) {
    ESP_LOGI(TAG, "Waiting ACK (timeout %ds)...",
             ACK_TIMEOUT_MS / 1000);

    uint8_t rx_buf[64];
    uint8_t rx_len = 0;
    int8_t  rssi   = 0;

    esp_err_t ret = sx1278_receive(dev, rx_buf,
                                    &rx_len, &rssi,
                                    ACK_TIMEOUT_MS);

    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "ACK timeout");
        return ESP_ERR_TIMEOUT;
    }
    if (ret == ESP_ERR_INVALID_CRC) {
        ESP_LOGW(TAG, "ACK CRC error");
        return ret;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RX error: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    if (rx_len >= sizeof(rx_buf))
        rx_len = sizeof(rx_buf) - 1;
    rx_buf[rx_len] = '\0';

    ESP_LOGI(TAG, "RX [%d bytes RSSI=%ddBm]: %s",
             rx_len, rssi, (char *)rx_buf);

    if (rx_buf[0] != PKT_TYPE_ACK) {
        ESP_LOGW(TAG, "Not an ACK");
        return ESP_ERR_INVALID_RESPONSE;
    }

    char *ptr = (char *)rx_buf;
    ptr = strchr(ptr, ','); if (!ptr) goto err; ptr++;

    uint8_t rx_node = (uint8_t)atoi(ptr);
    if (rx_node != node_id) {
        ESP_LOGW(TAG, "ACK for node %d not mine",
                 rx_node);
        return ESP_ERR_NOT_FOUND;
    }

    ptr = strchr(ptr, ','); if (!ptr) goto err; ptr++;
    uint8_t rx_stage = (uint8_t)atoi(ptr);

    ptr = strchr(ptr, ','); if (!ptr) goto err; ptr++;
    uint16_t rx_seq = (uint16_t)atoi(ptr);

    if (rx_stage > STAGE_RAIN) {
        ESP_LOGW(TAG, "Invalid stage %d", rx_stage);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG,
             "ACK OK Node=%d Stage=%s Seq=%d RSSI=%ddBm",
             rx_node, STAGE_NAMES[rx_stage],
             rx_seq, rssi);

    *stage_out = (fire_stage_t)rx_stage;
    return ESP_OK;

err:
    ESP_LOGE(TAG, "ACK parse error");
    return ESP_ERR_INVALID_RESPONSE;
}