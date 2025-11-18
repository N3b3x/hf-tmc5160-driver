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
for high torque, StallGuard2 for stall detection, and encoder support for closed-loop control. The driver provides
hardware-agnostic communication interfaces, allowing it to run on any platform (ESP32, STM32, etc.) with SPI or UART.

The driver exposes the full register interface, providing access to all TMC5160 registers through an intuitive C++ API.
It supports ramp control (positioning, velocity, hold modes), motor current configuration, chopper settings, stealthChop
PWM mode, encoder integration, and comprehensive protection systems.

## ✨ Features

- ✅ **Ramp Control**: Positioning, velocity, and hold modes with configurable acceleration profiles
- ✅ **Current Control**: Configurable run and hold currents with global scaler
- ✅ **Chopper Modes**: spreadCycle and stealthChop operation modes
- ✅ **StealthChop**: Silent operation with automatic PWM amplitude scaling
- ✅ **StallGuard2**: Stall detection and prevention with configurable thresholds
- ✅ **Encoder Support**: Closed-loop control with encoder feedback and deviation detection
- ✅ **Protection Systems**: Short circuit, overtemperature, and overvoltage protection
- ✅ **Hardware Agnostic**: SPI or UART interface for platform independence
- ✅ **Modern C++17**: Type-safe API with RAII principles
- ✅ **Zero Overhead**: CRTP-based design for compile-time polymorphism

## 🚀 Quick Start

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

For complete API documentation, see [docs/api_reference.md](docs/api_reference.md).

## 📊 Examples

For ESP32 examples, see the [examples/esp32](examples/esp32/) directory.

Detailed example walkthroughs are available in [docs/examples.md](docs/examples.md).

## 📚 Documentation

For complete documentation, see the [docs directory](docs/index.md).

## 🤝 Contributing

Pull requests and suggestions are welcome! Please follow the existing code style and include tests for new features.

## 📄 License

This project is licensed under the **GNU General Public License v3.0**.
See the [LICENSE](LICENSE) file for details.

