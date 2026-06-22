#include "sd_logger.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "sdmmc_cmd.h"
#include <stdio.h>
#include <string.h>

static const char    *TAG    = "SD";
static const char    *MOUNT  = "/sdcard";
static bool           mounted = false;
static sdmmc_card_t  *card    = NULL;

esp_err_t sd_logger_init(void) {
    spi_bus_config_t bus = {
        .mosi_io_num     = SD_MOSI,
        .miso_io_num     = SD_MISO,
        .sclk_io_num     = SD_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus,
                                        SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_CS;
    slot.host_id = SPI3_HOST;

    ret = esp_vfs_fat_sdspi_mount(MOUNT, &host, &slot,
                                   &mount_cfg, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        return ret;
    }

    sdmmc_card_print_info(stdout, card);
    mounted = true;
    ESP_LOGI(TAG, "SD mounted at %s", MOUNT);
    return ESP_OK;
}

esp_err_t sd_logger_write(const sd_log_entry_t *entry,
                           uint8_t node_id) {
    if (!mounted) {
        ESP_LOGW(TAG, "SD not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    char path[48];
    snprintf(path, sizeof(path),
             "%s/edge_node_%d.csv", MOUNT, node_id);

    // Write header if new file
    FILE *f = fopen(path, "r");
    bool need_header = (f == NULL);
    if (f) fclose(f);

    f = fopen(path, "a");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s", path);
        return ESP_FAIL;
    }

    // CSV header matches all fields including battery_voltage
    if (need_header) {
        fprintf(f,
            "Boot,Seq,NodeID,"
            "T680,H680,P680,Gas,"
            "T280,H280,P280,"
            "PM1.0,PM2.5,PM10,"
            "SOC,Voltage,"
            "Stage\n");
    }

    fprintf(f,
        "%lu,%d,%d,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%d,%d,%d,"
        "%d,%.3f,"
        "%d\n",
        (unsigned long)entry->boot,
        entry->seq,
        node_id,
        entry->bme680_temp,
        entry->bme680_humidity,
        entry->bme680_pressure,
        entry->bme680_gas,
        entry->bme280.temperature,
        entry->bme280.humidity,
        entry->bme280.pressure,
        entry->pms.pm1_0,
        entry->pms.pm2_5,
        entry->pms.pm10,
        entry->soc,
        entry->battery_voltage,
        (int)entry->stage);

    fclose(f);
    ESP_LOGI(TAG, "Logged → %s", path);
    return ESP_OK;
}

void sd_logger_deinit(void) {
    if (mounted) {
        esp_vfs_fat_sdcard_unmount(MOUNT, card);
        spi_bus_free(SPI3_HOST);
        mounted = false;
        ESP_LOGI(TAG, "SD unmounted");
    }
}