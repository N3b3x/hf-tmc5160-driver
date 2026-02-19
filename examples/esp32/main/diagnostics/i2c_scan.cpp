/**
 * @file i2c_scan.cpp
 * @brief Simple I2C bus scanner for the OLED UI board wiring.
 *
 * Uses the same pins as the fatigue_test_espnow_ui app to probe all 7-bit
 * addresses (0x03-0x77) and logs any devices that acknowledge.
 */

#include <cinttypes>
#include <cstdio>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmc51x0.hpp"

static const char* TAG = "I2C_SCAN";

static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_5;
static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_23;

// Scan frequencies: 400 kHz (fast mode) and 100 kHz (standard mode)
static constexpr uint32_t I2C_FREQ_FAST = 400000;  // 400 kHz fast mode
static constexpr uint32_t I2C_FREQ_STD  = 100000;  // 100 kHz standard mode

static esp_err_t scan_address(i2c_master_bus_handle_t bus,
                              uint8_t addr,
                              TickType_t timeout_ticks,
                              uint32_t freq_hz) {
    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = freq_hz;

    i2c_master_dev_handle_t dev_handle;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Try to read 1 byte - this will send address + read bit and check for ACK
    // Many I2C devices will ACK the address even if they can't provide data
    uint8_t dummy_byte = 0;
    err = i2c_master_receive(dev_handle, &dummy_byte, 1, timeout_ticks);
    
    // If read fails, try a write - some devices only respond to writes
    if (err != ESP_OK) {
        uint8_t dummy_write = 0;
        err = i2c_master_transmit(dev_handle, &dummy_write, 1, timeout_ticks);
    }

    esp_err_t rm_err = i2c_master_bus_rm_device(dev_handle);
    if (rm_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to remove device 0x%02X: %s", addr, esp_err_to_name(rm_err));
    }

    return err;
}

static void scan_at_frequency(i2c_master_bus_handle_t bus_handle, uint32_t freq_hz) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Scanning at %" PRIu32 " Hz ===", freq_hz);
    ESP_LOGI(TAG, "Scanning I2C addresses 0x00 to 0x7F (0-127)...");
    
    const TickType_t timeout_ticks = pdMS_TO_TICKS(200);  // Increased timeout for better detection
    int found = 0;

    for (uint16_t addr = 0x00; addr <= 0x7F; ++addr) {
        esp_err_t err = scan_address(bus_handle, addr, timeout_ticks, freq_hz);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ Device detected at 0x%02X", addr);
            ++found;
        } else {
            ESP_LOGD(TAG, "  . No response from 0x%02X", addr);
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "  No devices detected at %" PRIu32 " Hz", freq_hz);
    } else {
        ESP_LOGI(TAG, "  Found %d device(s) at %" PRIu32 " Hz", found, freq_hz);
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "I2C Scanner for OLED UI Board");
    ESP_LOGI(TAG, "Pins: SDA=GPIO%d, SCL=GPIO%d", 
             static_cast<int>(I2C_SDA_PIN),
             static_cast<int>(I2C_SCL_PIN));
    ESP_LOGI(TAG, "Will scan at both 400 kHz (fast) and 100 kHz (standard) speeds");
    ESP_LOGI(TAG, "Driver version: %s", tmc51x0::GetDriverVersion());

    i2c_master_bus_config_t bus_cfg{};
    bus_cfg.i2c_port = I2C_PORT;
    bus_cfg.sda_io_num = I2C_SDA_PIN;
    bus_cfg.scl_io_num = I2C_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle{};
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return;
    }

    // Scan at 400 kHz (fast mode)
    scan_at_frequency(bus_handle, I2C_FREQ_FAST);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Scan at 100 kHz (standard mode)
    scan_at_frequency(bus_handle, I2C_FREQ_STD);

    err = i2c_del_master_bus(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "I2C scan complete.");
    ESP_LOGI(TAG, "Devices found at 400 kHz work in fast mode.");
    ESP_LOGI(TAG, "Devices ONLY found at 100 kHz need standard mode.");

    // Keep the task alive briefly to allow log drain
    vTaskDelay(pdMS_TO_TICKS(1000));
}
