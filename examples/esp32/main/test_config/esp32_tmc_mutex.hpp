/**
 * @file esp32_tmc_mutex.hpp
 * @brief ESP32 FreeRTOS mutex wrapper for TMC51x0 driver thread safety
 * 
 * Provides RAII mutex classes for protecting TMC51x0 driver access in multi-threaded environments.
 * Uses FreeRTOS semaphores internally.
 * 
 * This is a shared utility located in test_config/ for use across all ESP32 examples.
 * Include this file when you need thread-safe access to TMC51x0 driver instances.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

/**
 * @brief ESP32 FreeRTOS mutex wrapper for TMC51x0 driver
 * 
 * RAII wrapper around FreeRTOS mutex (semaphore) for thread-safe TMC driver access.
 * Automatically creates mutex on construction and deletes on destruction.
 */
class Esp32TmcMutex {
public:
    Esp32TmcMutex() noexcept : handle_(xSemaphoreCreateMutex()) {
        if (handle_ == nullptr) {
            ESP_LOGE("Esp32TmcMutex", "Failed to create mutex");
        }
    }
    
    ~Esp32TmcMutex() noexcept {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
            handle_ = nullptr;
        }
    }
    
    // Non-copyable
    Esp32TmcMutex(const Esp32TmcMutex&) = delete;
    Esp32TmcMutex& operator=(const Esp32TmcMutex&) = delete;
    
    /**
     * @brief Get native FreeRTOS semaphore handle
     */
    SemaphoreHandle_t native_handle() const noexcept { return handle_; }
    
    /**
     * @brief Check if mutex is valid
     */
    bool is_valid() const noexcept { return handle_ != nullptr; }
    
private:
    SemaphoreHandle_t handle_;
};

/**
 * @brief RAII mutex guard for automatic lock/unlock
 * 
 * Automatically locks mutex on construction and unlocks on destruction.
 * Provides exception-safe mutex locking.
 */
class TmcMutexGuard {
public:
    /**
     * @brief Lock mutex (blocks until acquired)
     */
    explicit TmcMutexGuard(Esp32TmcMutex& mutex) noexcept : mutex_(mutex), locked_(false) {
        if (mutex_.is_valid()) {
            SemaphoreHandle_t handle = mutex_.native_handle();
            if (xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE) {
                locked_ = true;
            }
        }
    }
    
    /**
     * @brief Unlock mutex (automatically called on destruction)
     */
    ~TmcMutexGuard() noexcept { unlock(); }
    
    /**
     * @brief Manually unlock mutex
     */
    void unlock() noexcept {
        if (locked_ && mutex_.is_valid()) {
            SemaphoreHandle_t handle = mutex_.native_handle();
            xSemaphoreGive(handle);
            locked_ = false;
        }
    }
    
    /**
     * @brief Check if mutex is currently locked by this guard
     */
    bool is_locked() const noexcept { return locked_; }
    
private:
    Esp32TmcMutex& mutex_;
    bool locked_;
};

