---
layout: default
title: "HardFOC TMC5160 Driver"
description: "C++17 hardware-agnostic driver for Trinamic TMC5160 stepper motor controller with advanced features"
nav_order: 1
permalink: /
---

# HF-TMC5160 Driver
**C++17 hardware-agnostic driver for Trinamic TMC5160 stepper motor controller with advanced features**

[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17.html)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![CI](https://github.com/N3b3x/hf-tmc5160-driver/actions/workflows/esp32-examples-build-ci.yml/badge.svg?branch=main)](https://github.com/N3b3x/hf-tmc5160-driver/actions/workflows/esp32-examples-build-ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://github.com/N3b3x/hf-tmc5160-driver/tree/main/docs)

## 📚 Table of Contents
1. [Overview](#-overview)
2. [Features](#-features)
3. [Quick Start](#-quick-start)
4. [Installation](#-installation)
5. [API Reference](#-api-reference)
6. [Examples](#-examples)
7. [Documentation](#-documentation)
8. [Contributing](#-contributing)
9. [License](#-license)

## 📦 Overview

> **📖 [📚🌐 Complete Documentation](docs/index.md)** -
> Interactive guides, examples, and step-by-step tutorials

**HF-TMC5160** is a portable C++17 driver for the **Trinamic TMC5160** stepper motor controller IC. The TMC5160 is a
sophisticated stepper motor driver supporting advanced features including stealthChop for silent operation, spreadCycle
for high torque, StallGuard2 for stall detection, and encoder support for closed-loop control. 

This driver provides **complete feature coverage** of all 47 TMC5160 registers with 72+ public API methods organized into
intuitive subsystems. It supports **multi-chip communication** via SPI daisy chaining and UART multi-node addressing,
**physical unit conversions** (mm, degrees, RPM), **sensorless homing**, and many other advanced features not found in
other TMC5160 drivers. The driver provides hardware-agnostic communication interfaces, allowing it to run on any platform
(ESP32, STM32, etc.) with SPI or UART.

The driver exposes the full register interface, providing access to all 47 TMC5160 registers through an intuitive C++ API.
It supports ramp control (positioning, velocity, hold modes), motor current configuration, chopper settings, stealthChop
PWM mode, encoder integration, comprehensive protection systems, and advanced multi-chip communication.

## ✨ Features

### Core Motor Control
- ✅ **Ramp Control**: Positioning, velocity, and hold modes with configurable acceleration profiles
- ✅ **Current Control**: Configurable run and hold currents with global scaler
- ✅ **Chopper Modes**: spreadCycle and stealthChop operation modes
- ✅ **StealthChop**: Silent operation with automatic PWM amplitude scaling
- ✅ **StallGuard2**: Stall detection and prevention with configurable thresholds
- ✅ **Encoder Support**: Closed-loop control with encoder feedback and deviation detection
- ✅ **Protection Systems**: Short circuit, overtemperature, and overvoltage protection

### Multi-Chip Communication
- ✅ **SPI Daisy Chaining**: Connect multiple TMC5160 chips on a single SPI bus with automatic chain length detection
- ✅ **TMC5160DaisyChain Class**: High-level manager for multiple drivers with dynamic device management
- ✅ **UART Multi-Node**: Support for up to 255 devices on a single UART bus with slave addressing

### Advanced Features
- ✅ **Unit Conversions**: Physical unit support (millimeters, degrees, RPM, belt teeth) with convenience methods
- ✅ **Sensorless Homing**: StallGuard2-based sensorless homing for endstop-free operation
- ✅ **CoolStep**: Automatic current reduction when load is low for power efficiency
- ✅ **dcStep**: Automatic commutation for DC motor-like operation
- ✅ **Microstep Lookup Tables**: Custom microstep interpolation for optimized motion profiles
- ✅ **Lost Steps Detection**: Monitor and detect step loss during operation
- ✅ **OTP Programming**: One-time programmable memory read/write support
- ✅ **Factory Configuration**: Read factory calibration and configuration data

### Platform & Architecture
- ✅ **Hardware Agnostic**: SPI or UART interface for platform independence
- ✅ **Modern C++17**: Type-safe API with RAII principles
- ✅ **Zero Overhead**: CRTP-based design for compile-time polymorphism
- ✅ **Complete Register Coverage**: All 47 registers (0x00-0x73) with 72+ public API methods

## 🚀 Quick Start

### Single Motor Setup

```cpp
#include "inc/tmc5160.hpp"

// 1. Implement the communication interface (see platform_integration.md)
class MySPI : public tmc5160::SpiCommInterface<MySPI> {
    // ... implement required methods
};

// 2. Create driver instance
MySPI spi;
tmc5160::TMC5160 driver(spi);

// 3. Initialize driver
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;  // Run current (0-31)
cfg.motor.ihold = 10; // Hold current (0-31)
driver.Initialize(cfg);

// 4. Configure and start motor
driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
driver.rampControl.SetTargetPosition(1000);
driver.rampControl.SetMaxSpeed(1000.0f);
driver.rampControl.SetAcceleration(500.0f);
driver.motorControl.Enable();
```

### Multi-Motor Daisy Chain Setup

```cpp
#include "inc/tmc5160.hpp"
#include "inc/tmc5160_daisy_chain.hpp"

// 1. Create SPI communication interface (shared by all devices)
MySPI spi;
spi.Initialize();

// 2. Create daisy-chain manager with 3 onboard devices
tmc5160::TMC5160DaisyChain<MySPI, 5> chain(spi, 3, 12'000'000);

// 3. Initialize all devices
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;
cfg.motor.ihold = 10;
chain.InitializeAll(cfg);

// 4. Access individual motors
auto& x_axis = chain[0];
auto& y_axis = chain[1];
auto& z_axis = chain[2];

x_axis.rampControl.SetTargetPosition(1000);
y_axis.rampControl.SetMaxSpeed(500.0f);
z_axis.motorControl.Enable();
```

### Using Physical Units

```cpp
#include "inc/tmc5160.hpp"
#include "inc/tmc5160_units.hpp"

// Motor: 200 steps/rev, Lead screw: 2mm pitch
constexpr uint16_t STEPS_PER_REV = 200;
constexpr float LEAD_SCREW_PITCH_MM = 2.0f;

// Move 10mm using convenience method
driver.rampControl.SetTargetPositionMm(10.0f, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);

// Set speed in RPM
driver.rampControl.SetMaxSpeedRpm(100.0f, STEPS_PER_REV);

// Or use conversion functions directly
int32_t steps = tmc5160::MmToSteps(10.0f, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
driver.rampControl.SetTargetPosition(steps);
```

For detailed setup, see [Installation](docs/installation.md) and [Quick Start Guide](docs/quickstart.md).

## 🔧 Installation

1. **Clone or copy** the driver files into your project
2. **Implement the communication interface** for your platform (see [Platform Integration](docs/platform_integration.md))
3. **Include the header** in your code:
   ```cpp
   #include "inc/tmc5160.hpp"
   ```
4. Compile with a **C++17** or newer compiler

For detailed installation instructions, see [docs/installation.md](docs/installation.md).

## 📖 API Reference

### Core Methods

| Method | Description |
|--------|-------------|
| `Initialize()` | Initialize the driver with configuration |
| `rampControl.SetRampMode()` | Set ramp mode (positioning, velocity, hold) |
| `rampControl.SetTargetPosition()` | Set target position for positioning mode |
| `rampControl.SetMaxSpeed()` | Set maximum velocity |
| `rampControl.SetAcceleration()` | Set acceleration/deceleration |
| `motorControl.Enable()` | Enable the motor driver |
| `motorControl.Disable()` | Disable the motor driver |
| `motorControl.SetCurrent()` | Set run and hold currents |
| `diagnostics.GetStatus()` | Get driver status and error conditions |

### Multi-Chip Methods

| Method | Description |
|--------|-------------|
| `SetDaisyChainPosition()` | Set position in SPI daisy chain |
| `SetDaisyChainLength()` | Configure total chain length for optimal response extraction |
| `TMC5160DaisyChain` | High-level manager for multiple drivers on shared SPI bus |

### Unit Conversion Methods

| Method | Description |
|--------|-------------|
| `rampControl.SetTargetPositionMm()` | Set target position in millimeters |
| `rampControl.SetMaxSpeedRpm()` | Set maximum speed in RPM |
| `MmToSteps()` / `StepsToMm()` | Convert between millimeters and steps |
| `RpmToStepsPerSec()` / `StepsPerSecToRpm()` | Convert between RPM and steps/s |
| `DegreesToSteps()` / `StepsToDegrees()` | Convert between degrees and steps |

### Advanced Features

| Method | Description |
|--------|-------------|
| `PerformSensorlessHoming()` | Sensorless homing using StallGuard2 |
| `ConfigureCoolStep()` | Configure automatic current reduction |
| `ConfigureDcStep()` | Configure dcStep automatic commutation |
| `SetMicrostepLookupTable()` | Configure custom microstep interpolation |
| `GetLostSteps()` | Read lost steps counter |
| `ReadOtpConfig()` | Read OTP memory |
| `ReadFactoryConfig()` | Read factory calibration data |

For complete API documentation, see [docs/api_reference.md](docs/api_reference.md).

## 📊 Examples

Comprehensive ESP32 examples are available in the [examples/esp32](examples/esp32/) directory, including:

- **Core Functionality**: Basic motor control, ramp modes, current configuration
- **Multi-Chip Communication**: 
  - `spi_daisy_chain_comprehensive_test.cpp` - SPI daisy chaining with multiple motors
  - `uart_multi_node_comprehensive_test.cpp` - UART multi-node addressing
- **Advanced Features**:
  - `encoder_comprehensive_test.cpp` - Encoder integration and closed-loop control
  - `diagnostics_comprehensive_test.cpp` - Comprehensive diagnostics and status monitoring
  - `protection_comprehensive_test.cpp` - Protection system configuration and monitoring
- **Motor Control**:
  - `motor_control_comprehensive_test.cpp` - Advanced motor control features
  - `ramp_control_comprehensive_test.cpp` - Ramp control and motion profiles
  - `bounds_finding_sinuous_motion.cpp` - Advanced motion control with bounds finding

Detailed example walkthroughs are available in [docs/examples.md](docs/examples.md).

## 📚 Documentation

For complete documentation, see the [docs directory](docs/index.md).

## 🤝 Contributing

Pull requests and suggestions are welcome! Please follow the existing code style and include tests for new features.

## 📄 License

This project is licensed under the **GNU General Public License v3.0**.
See the [LICENSE](LICENSE) file for details.

