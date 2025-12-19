---
layout: default
title: "🔧 Platform Integration"
description: "Implement the CRTP communication interface for your platform"
nav_order: 4
parent: "📚 Documentation"
permalink: /docs/platform_integration/
---

# Platform Integration Guide

This guide explains how to implement the hardware abstraction interface for the TMC51x0 driver (TMC5130 & TMC5160) on your platform.

## Understanding CRTP (Curiously Recurring Template Pattern)

The TMC51x0 driver (TMC5130 & TMC5160) uses **CRTP** (Curiously Recurring Template Pattern) for hardware abstraction. This design choice
provides several critical benefits for embedded systems:

### Why CRTP Instead of Virtual Functions?

#### 1. **Zero Runtime Overhead**
- **Virtual functions**: Require a vtable lookup (indirect call) = ~5-10 CPU cycles overhead per call
- **CRTP**: Direct function calls = 0 overhead, compiler can inline
- **Impact**: In time-critical embedded code, this matters significantly

#### 2. **Compile-Time Polymorphism**
- **Virtual functions**: Runtime dispatch - the compiler cannot optimize across the abstraction boundary
- **CRTP**: Compile-time dispatch - full optimization, dead code elimination, constant propagation
- **Impact**: Smaller code size, faster execution

#### 3. **Memory Efficiency**
- **Virtual functions**: Each object needs a vtable pointer (4-8 bytes)
- **CRTP**: No vtable pointer needed
- **Impact**: Critical in memory-constrained systems

## SPI Interface Implementation

### ESP32 (ESP-IDF) Example

```cpp
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "inc/tmc51x0.hpp"

class Esp32SPI : public tmc51x0::SpiCommInterface<Esp32SPI> {
private:
    spi_device_handle_t spi_device_;
    gpio_num_t cs_pin_;
    gpio_num_t en_pin_;
    gpio_num_t dir_pin_;
    gpio_num_t step_pin_;

public:
    Esp32SPI(spi_host_device_t host, gpio_num_t cs_pin, gpio_num_t en_pin,
             gpio_num_t dir_pin, gpio_num_t step_pin)
        : SpiCommInterface(true, true, true), // EN, DIR, STEP active high
          cs_pin_(cs_pin), en_pin_(en_pin), dir_pin_(dir_pin), step_pin_(step_pin) {
        spi_device_interface_config_t dev_config = {};
        dev_config.clock_speed_hz = 4 * 1000 * 1000; // 4 MHz
        dev_config.mode = 3; // SPI Mode 3
        dev_config.spics_io_num = cs_pin_;
        dev_config.queue_size = 1;
        
        spi_bus_add_device(host, &dev_config, &spi_device_);
        
        // Configure GPIO pins
        gpio_set_direction(en_pin_, GPIO_MODE_OUTPUT);
        gpio_set_direction(dir_pin_, GPIO_MODE_OUTPUT);
        gpio_set_direction(step_pin_, GPIO_MODE_OUTPUT);
    }

    CommMode GetMode() const noexcept {
        return CommMode::SPI;
    }

    bool SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) noexcept {
        spi_transaction_t trans = {};
        trans.length = length * 8;
        trans.tx_buffer = tx;
        trans.rx_buffer = rx;
        
        esp_err_t ret = spi_device_transmit(spi_device_, &trans);
        return ret == ESP_OK;
    }

    bool GpioSet(TMC51x0CtrlPin pin, GpioSignal signal) noexcept {
        gpio_num_t gpio_pin;
        switch (pin) {
            case TMC51x0CtrlPin::EN:
                gpio_pin = en_pin_;
                break;
            case TMC51x0CtrlPin::DIR:
                gpio_pin = dir_pin_;
                break;
            case TMC51x0CtrlPin::STEP:
                gpio_pin = step_pin_;
                break;
            default:
                return false;
        }
        
        bool level = SignalToGpioLevel(pin, signal);
        gpio_set_level(gpio_pin, level ? 1 : 0);
        return true;
    }

    bool GpioRead(TMC51x0CtrlPin pin, GpioSignal& signal) noexcept {
        // TMC5160 doesn't have input pins for reading, but implement for completeness
        return false;
    }

    void DebugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        esp_log_level_t esp_level;
        switch (level) {
            case 0: esp_level = ESP_LOG_ERROR; break;
            case 1: esp_level = ESP_LOG_WARN; break;
            case 2: esp_level = ESP_LOG_INFO; break;
            case 3: esp_level = ESP_LOG_DEBUG; break;
            default: esp_level = ESP_LOG_VERBOSE; break;
        }
        esp_log_writev(esp_level, tag, format, args);
    }

    void DelayMs(uint32_t ms) noexcept {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void DelayUs(uint32_t us) noexcept {
        esp_rom_delay_us(us);
    }
};
```

### STM32 (HAL) Example

```cpp
#include "stm32f4xx_hal.h"
#include "inc/tmc51x0.hpp"

class STM32SPI : public tmc51x0::SpiCommInterface<STM32SPI> {
private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
    GPIO_TypeDef* en_port_;
    uint16_t en_pin_;

public:
    STM32SPI(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
             GPIO_TypeDef* en_port, uint16_t en_pin)
        : SpiCommInterface(true, true, true),
          hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin),
          en_port_(en_port), en_pin_(en_pin) {}

    CommMode GetMode() const noexcept {
        return CommMode::SPI;
    }

    bool SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) noexcept {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, 
            const_cast<uint8_t*>(tx), const_cast<uint8_t*>(rx), length, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        return status == HAL_OK;
    }

    bool GpioSet(TMC51x0CtrlPin pin, GpioSignal signal) noexcept {
        GPIO_TypeDef* port;
        uint16_t gpio_pin;
        switch (pin) {
            case TMC51x0CtrlPin::EN:
                port = en_port_;
                gpio_pin = en_pin_;
                break;
            default:
                return false;
        }
        
        bool level = SignalToGpioLevel(pin, signal);
        HAL_GPIO_WritePin(port, gpio_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return true;
    }

    bool GpioRead(TMC51x0CtrlPin pin, GpioSignal& signal) noexcept {
        return false;
    }

    void DebugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        // Implement your logging
    }

    void DelayMs(uint32_t ms) noexcept {
        HAL_Delay(ms);
    }

    void DelayUs(uint32_t us) noexcept {
        // STM32 HAL doesn't have microsecond delay, use DWT or timer
        // For now, approximate with HAL_Delay
        if (us >= 1000) {
            HAL_Delay(us / 1000);
        }
    }
};
```

## UART Interface Implementation

### ESP32 UART Example

```cpp
#include "driver/uart.h"
#include "inc/tmc51x0.hpp"

class Esp32UART : public tmc51x0::UartCommInterface<Esp32UART> {
private:
    uart_port_t uart_num_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    gpio_num_t txen_pin_;
    gpio_num_t tmc_power_en_pin_{GPIO_NUM_NC}; // optional: load-switch enable for TMC VIO

public:
    Esp32UART(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
              gpio_num_t txen_pin, uint8_t node_address,
              gpio_num_t tmc_power_en_pin = GPIO_NUM_NC)
        : UartCommInterface(true, true, true, node_address),
          uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), txen_pin_(txen_pin),
          tmc_power_en_pin_(tmc_power_en_pin) {
        uart_config_t uart_config = {};
        uart_config.baud_rate = 500000;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        
        uart_param_config(uart_num_, &uart_config);
        uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(uart_num_, 1024, 1024, 0, NULL, 0);
        
        gpio_set_direction(txen_pin_, GPIO_MODE_OUTPUT);

        // Optional: power-enable control for true hard reset (power cycle)
        if (tmc_power_en_pin_ != GPIO_NUM_NC) {
            gpio_set_direction(tmc_power_en_pin_, GPIO_MODE_OUTPUT);
            gpio_set_level(tmc_power_en_pin_, 1); // power on by default
        }
    }

    CommMode GetMode() const noexcept {
        return CommMode::UART;
    }

    bool UartSend(const uint8_t* data, size_t length) noexcept {
        gpio_set_level(txen_pin_, 1); // Enable transmitter
        int bytes_written = uart_write_bytes(uart_num_, data, length);
        uart_wait_tx_done(uart_num_, portMAX_DELAY);
        gpio_set_level(txen_pin_, 0); // Disable transmitter
        return bytes_written == length;
    }

    // Optional power control hook (enables TMC51x0::HardReset() to power-cycle)
    tmc51x0::Result<void> SetPowerEnabled(bool enabled) noexcept {
        if (tmc_power_en_pin_ == GPIO_NUM_NC) {
            return tmc51x0::Result<void>(tmc51x0::ErrorCode::UNSUPPORTED);
        }
        gpio_set_level(tmc_power_en_pin_, enabled ? 1 : 0);
        return tmc51x0::Result<void>();
    }

    bool UartReceive(uint8_t* data, size_t length) noexcept {
        int bytes_read = uart_read_bytes(uart_num_, data, length, pdMS_TO_TICKS(100));
        return bytes_read == length;
    }

    bool GpioSet(TMC51x0CtrlPin pin, GpioSignal signal) noexcept {
        return true; // UART mode doesn't use GPIO pins
    }

    bool GpioRead(TMC51x0CtrlPin pin, GpioSignal& signal) noexcept {
        return false;
    }

    void DebugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        esp_log_writev(ESP_LOG_DEBUG, tag, format, args);
    }

    void DelayMs(uint32_t ms) noexcept {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void DelayUs(uint32_t us) noexcept {
        esp_rom_delay_us(us);
    }
};
```

## Required Methods Summary

### SPI Interface

You must implement:
- `GetMode()` - Return `CommMode::SPI`
- `SpiTransfer()` - Perform SPI transfer
- `GpioSet()` - Set GPIO pins (EN, DIR, STEP)
- `GpioRead()` - Read GPIO pins (if needed)
- `DebugLog()` - Debug logging (optional)
- `DelayMs()` - Millisecond delay
- `DelayUs()` - Microsecond delay

### UART Interface

You must implement:
- `GetMode()` - Return `CommMode::UART`
- `UartSend()` - Send bytes via UART
- `UartReceive()` - Receive bytes via UART
- `GpioSet()` - Set GPIO pins (if needed)
- `GpioRead()` - Read GPIO pins (if needed)
- `DebugLog()` - Debug logging (optional)
- `DelayMs()` - Millisecond delay
- `DelayUs()` - Microsecond delay

## Next Steps

- Review the [Configuration](configuration.md) guide for driver settings
- Check [Examples](examples.md) for complete working examples
- See [API Reference](api_reference.md) for all available methods

---

**Navigation**
⬅️ [Hardware Setup](hardware_setup.md) | [Next: Configuration ➡️](configuration.md) | [Back to Index](index.md)

