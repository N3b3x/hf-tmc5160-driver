/**
 * @file Adafruit_I2CDevice.cpp
 * @brief ESP-IDF native I2C device implementation
 */

#include "Adafruit_I2CDevice.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG_I2C = "I2CDevice";

// Global I2C bus handle (shared across all devices on same bus)
static i2c_master_bus_handle_t s_global_i2c_bus = nullptr;
static bool s_bus_initialized = false;

// Default I2C pins (can be overridden via constructor or begin())
gpio_num_t Adafruit_I2CDevice::s_default_sda = GPIO_NUM_21;
gpio_num_t Adafruit_I2CDevice::s_default_scl = GPIO_NUM_22;
uint32_t Adafruit_I2CDevice::s_default_freq = 400000;

Adafruit_I2CDevice::Adafruit_I2CDevice(uint8_t addr, void* theWire)
    : addr_(addr), bus_handle_(nullptr), device_handle_(nullptr), 
      initialized_(false), sda_pin_(s_default_sda), scl_pin_(s_default_scl), 
      i2c_freq_(s_default_freq) {
    // If theWire is provided, use it (cast from void*)
    if (theWire) {
        bus_handle_ = static_cast<i2c_master_bus_handle_t>(theWire);
    }
}

Adafruit_I2CDevice::~Adafruit_I2CDevice() {
    end();
}

bool Adafruit_I2CDevice::initBus() {
    // If bus is already initialized, use it
    if (s_bus_initialized && s_global_i2c_bus) {
        bus_handle_ = s_global_i2c_bus;
        return true;
    }
    
    // Initialize I2C bus using configured pins
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_pin_,
        .scl_io_num = scl_pin_,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return false;
    }
    
    s_global_i2c_bus = bus_handle_;
    s_bus_initialized = true;
    ESP_LOGI(TAG_I2C, "I2C bus initialized: SDA=GPIO%d, SCL=GPIO%d", 
             sda_pin_, scl_pin_);
    return true;
}

bool Adafruit_I2CDevice::begin(bool addr_detect) {
    if (initialized_) {
        return true;
    }
    
    // Initialize I2C bus if needed
    if (!initBus()) {
        return false;
    }
    
    // Configure I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_,
        .scl_speed_hz = i2c_freq_,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(bus_handle_, &dev_cfg, &device_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "Failed to add I2C device (addr=0x%02X): %s", 
                 addr_, esp_err_to_name(ret));
        return false;
    }
    
    initialized_ = true;
    
    // Optional: Detect device
    if (addr_detect) {
        if (!detected()) {
            ESP_LOGW(TAG_I2C, "I2C device at 0x%02X not detected", addr_);
            // Don't fail - device might be slow to respond
        }
    }
    
    ESP_LOGI(TAG_I2C, "I2C device initialized: addr=0x%02X", addr_);
    return true;
}

void Adafruit_I2CDevice::end() {
    if (device_handle_) {
        i2c_master_bus_rm_device(device_handle_);
        device_handle_ = nullptr;
    }
    initialized_ = false;
}

bool Adafruit_I2CDevice::detected() {
    if (!initialized_ || !device_handle_) {
        return false;
    }
    
    // Try to read 1 byte (some devices support this, others need write-then-read)
    uint8_t dummy;
    esp_err_t ret = i2c_master_transmit_receive(device_handle_, nullptr, 0, &dummy, 1, 
                                                 pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        return true;
    }
    
    // Alternative: Try write (some devices ACK on write)
    ret = i2c_master_transmit(device_handle_, nullptr, 0, pdMS_TO_TICKS(100));
    return (ret == ESP_OK);
}

bool Adafruit_I2CDevice::read(uint8_t *buffer, size_t len, bool stop) {
    if (!initialized_ || !device_handle_ || !buffer || len == 0) {
        return false;
    }
    
    esp_err_t ret = i2c_master_receive(device_handle_, buffer, len, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "I2C read failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    return true;
}

bool Adafruit_I2CDevice::write(const uint8_t *buffer, size_t len, bool stop,
                               const uint8_t *prefix_buffer, size_t prefix_len) {
    if (!initialized_ || !device_handle_ || !buffer || len == 0) {
        return false;
    }
    
    // Combine prefix and data if prefix is provided
    if (prefix_buffer && prefix_len > 0) {
        uint8_t combined[prefix_len + len];
        memcpy(combined, prefix_buffer, prefix_len);
        memcpy(combined + prefix_len, buffer, len);
        
        esp_err_t ret = i2c_master_transmit(device_handle_, combined, 
                                            prefix_len + len, pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_I2C, "I2C write failed: %s", esp_err_to_name(ret));
            return false;
        }
    } else {
        esp_err_t ret = i2c_master_transmit(device_handle_, buffer, len, pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_I2C, "I2C write failed: %s", esp_err_to_name(ret));
            return false;
        }
    }
    
    return true;
}

bool Adafruit_I2CDevice::write_then_read(const uint8_t *write_buffer, size_t write_len,
                                         uint8_t *read_buffer, size_t read_len, bool stop) {
    if (!initialized_ || !device_handle_) {
        return false;
    }
    
    if (write_buffer && write_len > 0) {
        esp_err_t ret = i2c_master_transmit(device_handle_, write_buffer, write_len, 
                                            pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_I2C, "I2C write_then_read: write failed: %s", esp_err_to_name(ret));
            return false;
        }
    }
    
    if (read_buffer && read_len > 0) {
        esp_err_t ret = i2c_master_receive(device_handle_, read_buffer, read_len, 
                                          pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_I2C, "I2C write_then_read: read failed: %s", esp_err_to_name(ret));
            return false;
        }
    }
    
    return true;
}

bool Adafruit_I2CDevice::setSpeed(uint32_t desiredclk) {
    // I2C speed is set per device in ESP-IDF, but we can't change it dynamically
    // Store desired frequency for future use
    i2c_freq_ = desiredclk;
    return true;
}

