/**
 * @file sd_card.cpp
 * @brief SD card implementation
 */

#include "sd_card.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include <dirent.h>
#include <cstring>
#include <string>
#include <cstdio>

static const char* TAG_SD = "SDCard";

static bool s_mounted = false;
static const char* s_mount_point = SD_MOUNT_POINT;

bool SDCard::init()
{
    if (s_mounted) {
        return true;
    }

    ESP_LOGI(TAG_SD, "Initializing SD card...");

    // Configure SPI bus for SD card
    // ESP32-C6 only has SPI2_HOST, so we share the SPI bus with the display
    // Both devices use different CS pins, so they can share the same SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // Try to initialize SPI bus (may fail if already initialized by display)
    // ESP32-C6 only has SPI2_HOST
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG_SD, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    // Get default SDSPI host configuration
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    // Configure SD card slot
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = SPI2_HOST;  // ESP32-C6 only has SPI2_HOST

    // Mount filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false
    };

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount(s_mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_SD, "Failed to mount filesystem. Format SD card as FAT32.");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG_SD, "SD card not found. Check wiring.");
        } else {
            ESP_LOGE(TAG_SD, "Failed to mount SD card: %s", esp_err_to_name(ret));
        }
        return false;
    }

    // Print card info
    sdmmc_card_print_info(stdout, card);
    s_mounted = true;
    ESP_LOGI(TAG_SD, "SD card mounted successfully at %s", s_mount_point);
    return true;
}

void SDCard::deinit()
{
    if (!s_mounted) {
        return;
    }

    esp_vfs_fat_sdcard_unmount(s_mount_point, nullptr);
    spi_bus_free(SPI2_HOST);
    s_mounted = false;
    ESP_LOGI(TAG_SD, "SD card unmounted");
}

bool SDCard::isMounted()
{
    return s_mounted;
}

size_t SDCard::scanForImages(const char* directory, std::vector<std::string>& imageFiles)
{
    imageFiles.clear();

    if (!s_mounted) {
        ESP_LOGE(TAG_SD, "SD card not mounted");
        return 0;
    }

    DIR* dir = opendir(directory);
    if (dir == nullptr) {
        ESP_LOGE(TAG_SD, "Failed to open directory: %s", directory);
        return 0;
    }

    struct dirent* entry;
    size_t count = 0;

    while ((entry = readdir(dir)) != nullptr && count < MAX_IMAGE_FILES) {
        // Skip . and ..
        if (entry->d_name[0] == '.') {
            continue;
        }

        // Check file extension
        std::string filename(entry->d_name);
        bool isImage = false;
        for (size_t i = 0; i < NUM_IMAGE_EXTENSIONS; i++) {
            if (filename.length() >= strlen(IMAGE_EXTENSIONS[i])) {
                std::string ext = filename.substr(filename.length() - strlen(IMAGE_EXTENSIONS[i]));
                if (ext == IMAGE_EXTENSIONS[i]) {
                    isImage = true;
                    break;
                }
            }
        }

        if (isImage) {
            std::string fullPath = std::string(directory) + "/" + filename;
            imageFiles.push_back(fullPath);
            count++;
        }
    }

    closedir(dir);
    ESP_LOGI(TAG_SD, "Found %zu image files in %s", count, directory);
    return count;
}

int32_t SDCard::readFile(const char* filepath, uint8_t* buffer, size_t maxSize)
{
    if (!s_mounted || !buffer || maxSize == 0) {
        return -1;
    }

    FILE* file = fopen(filepath, "rb");
    if (file == nullptr) {
        ESP_LOGE(TAG_SD, "Failed to open file: %s", filepath);
        return -1;
    }

    size_t bytesRead = fread(buffer, 1, maxSize, file);
    fclose(file);

    return static_cast<int32_t>(bytesRead);
}

int32_t SDCard::getFileSize(const char* filepath)
{
    if (!s_mounted) {
        return -1;
    }

    FILE* file = fopen(filepath, "rb");
    if (file == nullptr) {
        return -1;
    }

    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fclose(file);

    return static_cast<int32_t>(size);
}

