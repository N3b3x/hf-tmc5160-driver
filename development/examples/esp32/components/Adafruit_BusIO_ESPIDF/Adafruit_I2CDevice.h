/**
 * @file Adafruit_I2CDevice.h
 * @brief ESP-IDF native I2C device wrapper for Adafruit libraries
 * 
 * Provides Adafruit_I2CDevice interface using ESP-IDF I2C master driver.
 * Compatible with Adafruit GFX and OLED libraries.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief I2C device wrapper compatible with Adafruit libraries
 */
class Adafruit_I2CDevice {
public:
    /**
     * @brief Constructor
     * @param addr I2C device address (7-bit, will be shifted left)
     * @param theWire I2C bus handle (i2c_master_bus_handle_t), can be nullptr
     */
    Adafruit_I2CDevice(uint8_t addr, void* theWire = nullptr);
    
    /**
     * @brief Destructor
     */
    ~Adafruit_I2CDevice();
    
    /**
     * @brief Get I2C device address
     * @return 7-bit I2C address
     */
    uint8_t address(void) const { return addr_; }
    
    /**
     * @brief Initialize I2C device
     * @param addr_detect If true, try to detect device on bus
     * @return true if successful, false otherwise
     */
    bool begin(bool addr_detect = true);
    
    /**
     * @brief Deinitialize I2C device
     */
    void end(void);
    
    /**
     * @brief Check if device is detected on bus
     * @return true if device responds, false otherwise
     */
    bool detected(void);
    
    /**
     * @brief Read data from I2C device
     * @param buffer Buffer to store read data
     * @param len Number of bytes to read
     * @param stop Send stop condition after read
     * @return true if successful, false otherwise
     */
    bool read(uint8_t *buffer, size_t len, bool stop = true);
    
    /**
     * @brief Write data to I2C device
     * @param buffer Data to write
     * @param len Number of bytes to write
     * @param stop Send stop condition after write
     * @param prefix_buffer Optional prefix data (e.g., register address)
     * @param prefix_len Length of prefix data
     * @return true if successful, false otherwise
     */
    bool write(const uint8_t *buffer, size_t len, bool stop = true,
               const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
    
    /**
     * @brief Write then read (for register-based devices)
     * @param write_buffer Data to write
     * @param write_len Number of bytes to write
     * @param read_buffer Buffer to store read data
     * @param read_len Number of bytes to read
     * @param stop Send stop condition
     * @return true if successful, false otherwise
     */
    bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                        uint8_t *read_buffer, size_t read_len,
                        bool stop = false);
    
    /**
     * @brief Set I2C bus speed (not implemented, uses bus default)
     * @param desiredclk Desired clock frequency in Hz
     * @return true if successful
     */
    bool setSpeed(uint32_t desiredclk);
    
    /**
     * @brief Get maximum buffer size for I2C transactions
     * @return Maximum buffer size (typically 512 bytes for ESP-IDF)
     */
    size_t maxBufferSize() const { return 512; }
    
    /**
     * @brief Get I2C device handle (for advanced usage)
     * @return i2c_master_dev_handle_t or nullptr if not initialized
     */
    i2c_master_dev_handle_t getHandle() const { return device_handle_; }
    
    /**
     * @brief Get I2C bus handle (for advanced usage)
     * @return i2c_master_bus_handle_t or nullptr if not initialized
     */
    i2c_master_bus_handle_t getBusHandle() const { return bus_handle_; }
    
    /**
     * @brief Set I2C pins (must be called before begin() if using non-default pins)
     * @param sda SDA GPIO pin
     * @param scl SCL GPIO pin
     */
    void setPins(gpio_num_t sda, gpio_num_t scl) {
        sda_pin_ = sda;
        scl_pin_ = scl;
    }
    
    /**
     * @brief Set I2C frequency (must be called before begin())
     * @param freq Frequency in Hz
     */
    void setFrequency(uint32_t freq) {
        i2c_freq_ = freq;
    }
    
    /**
     * @brief Set default I2C pins for all devices (static)
     * @param sda Default SDA GPIO pin
     * @param scl Default SCL GPIO pin
     */
    static void setDefaultPins(gpio_num_t sda, gpio_num_t scl) {
        s_default_sda = sda;
        s_default_scl = scl;
    }
    
    /**
     * @brief Set default I2C frequency for all devices (static)
     * @param freq Default frequency in Hz
     */
    static void setDefaultFrequency(uint32_t freq) {
        s_default_freq = freq;
    }

private:
    static gpio_num_t s_default_sda;
    static gpio_num_t s_default_scl;
    static uint32_t s_default_freq;
    uint8_t addr_;                              // 7-bit I2C address
    i2c_master_bus_handle_t bus_handle_;        // I2C bus handle
    i2c_master_dev_handle_t device_handle_;     // I2C device handle
    bool initialized_;                          // Initialization state
    gpio_num_t sda_pin_;                        // SDA GPIO pin
    gpio_num_t scl_pin_;                        // SCL GPIO pin
    uint32_t i2c_freq_;                         // I2C frequency
    
    // Initialize I2C bus if not already initialized
    bool initBus();
};
