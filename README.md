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

**HF-TMC5160** is a comprehensive, production-ready C++17 driver for the **Trinamic TMC5160** stepper motor controller IC. 
The TMC5160 is a sophisticated stepper motor driver supporting advanced features including stealthChop for silent operation, 
spreadCycle for high torque, StallGuard2 for stall detection, and encoder support for closed-loop control. 

This driver provides **complete feature coverage** of all 47 TMC5160 registers with **100+ public API methods** organized into 
intuitive, well-structured subsystems. It supports **multi-chip communication** via SPI daisy chaining and UART multi-node 
addressing, **physical unit conversions** (mm, degrees, RPM, belt teeth), **automatic parameter tuning**, **sensorless homing**, 
and many other advanced features not found in other TMC5160 drivers.

### Architecture & Design

The driver is built with a **subsystem-based architecture** that organizes functionality into logical groups, making it easy 
to find and use the features you need:

```cpp
tmc5160::TMC5160<MySPI> driver(spi);

// Organized subsystems for intuitive access
driver.rampControl      // Motion planning and positioning
driver.motorControl     // Current control and chopper modes
driver.encoder          // Encoder integration and closed-loop control
driver.diagnostics      // Status monitoring and StallGuard2 reading
driver.tuning           // Automatic parameter optimization
driver.homing           // Sensorless and switch-based homing
driver.protection       // Safety and protection systems
driver.communication    // Multi-chip communication setup
driver.printer          // Debug register printing
```

The driver uses **CRTP (Curiously Recurring Template Pattern)** for hardware-agnostic communication interfaces, allowing it 
to run on any platform (ESP32, STM32, Arduino, Raspberry Pi, etc.) with SPI or UART. This design provides **zero runtime 
overhead** while maintaining complete platform independence.

### Key Capabilities

- **Complete Register Coverage**: All 47 TMC5160 registers (0x00-0x73) accessible through type-safe C++ API
- **Multi-Chip Support**: SPI daisy chaining and UART multi-node addressing for controlling multiple motors
- **Unit Conversions**: Work with physical units (mm, degrees, RPM) instead of raw steps
- **Automatic Tuning**: Intelligent StallGuard2 threshold optimization with velocity range analysis
- **Sensorless Homing**: Endstop-free homing using StallGuard2 stall detection
- **Dual Chip Support**: Automatically detects and supports both TMC5130 and TMC5160 chips
- **Modern C++17**: Type-safe API with RAII principles and compile-time optimizations

## ✨ Features

### 🎯 Core Motor Control

#### RampControl Subsystem
- ✅ **Positioning Mode**: Precise position control with configurable acceleration profiles
- ✅ **Velocity Mode**: Continuous motion in positive or negative direction
- ✅ **Hold Mode**: Maintain position with configurable hold current
- ✅ **Advanced Ramp Profiles**: Multi-phase acceleration (A1, AMAX, D1) for smooth motion
- ✅ **Reference Switches**: Configurable endstops with latching and stop-on-switch modes
- ✅ **Unit-Aware API**: Set positions, speeds, and accelerations in steps, mm, degrees, or RPM

#### MotorControl Subsystem
- ✅ **Current Control**: Automatic calculation from motor specifications (rated current, sense resistor)
- ✅ **Run/Hold Currents**: Independent run and hold current settings with global scaler
- ✅ **Chopper Modes**: 
  - **spreadCycle**: High torque mode with configurable chopper settings
  - **stealthChop**: Silent operation with automatic PWM amplitude scaling
- ✅ **Mode Switching**: Automatic switching between stealthChop and spreadCycle based on velocity
- ✅ **Freewheeling**: Automatic freewheeling when motor is stopped

### 🔍 Diagnostics & Tuning

#### Diagnostics Subsystem
- ✅ **Status Monitoring**: Comprehensive driver status (overtemperature, short circuit, etc.)
- ✅ **StallGuard2 Reading**: Real-time load measurement (0-1023) for diagnostics
- ✅ **Open Load Detection**: Detect interrupted cables or connector issues
- ✅ **Lost Steps Counter**: Monitor step loss during operation
- ✅ **Register Access**: Direct read/write access to all 47 registers
- ✅ **Setup Verification**: Comprehensive startup verification and diagnostics

#### Tuning Subsystem ⭐ NEW
- ✅ **Automatic SGT Tuning**: Intelligent StallGuard2 threshold optimization
- ✅ **Velocity Range Analysis**: Finds optimal SGT for target velocity with min/max range testing
- ✅ **Comprehensive Results**: Returns detailed tuning results including actual achievable velocities
- ✅ **Target Velocity Priority**: Optimizes SGT primarily for your most important operating speed

### 🏠 Homing & Positioning

#### Homing Subsystem
- ✅ **Sensorless Homing**: Endstop-free homing using StallGuard2 stall detection
- ✅ **Switch-Based Homing**: Homing using reference switches/endstops
- ✅ **Automatic Settings Caching**: Preserves and restores driver settings during homing
- ✅ **Configurable Search Speed**: Adjustable homing speed and switch approach speed

### 🔄 Encoder Integration

#### Encoder Subsystem
- ✅ **Closed-Loop Control**: Encoder feedback for position verification
- ✅ **Deviation Detection**: Monitor and detect position deviation (step loss)
- ✅ **Automatic Compensation**: Optional automatic position correction
- ✅ **Encoder Configuration**: Flexible encoder setup (AB, ABZ, SPI, etc.)

### 🛡️ Protection Systems

#### Protection Subsystem
- ✅ **Short Circuit Protection**: Configurable voltage thresholds and timing
- ✅ **Overtemperature Protection**: Automatic shutdown on overtemperature
- ✅ **Overvoltage Protection**: Undervoltage charge pump monitoring
- ✅ **Driver Status**: Real-time monitoring of all protection flags

### 🔗 Multi-Chip Communication

#### Communication Subsystem
- ✅ **SPI Daisy Chaining**: Connect multiple TMC5160 chips on a single SPI bus
  - Automatic chain length detection
  - Position-based addressing (0, 1, 2, ...)
  - `TMC5160DaisyChain` helper class for easy management
- ✅ **UART Multi-Node**: Support for up to 255 devices on a single UART bus
  - Slave addressing (0-254)
  - Sequential programming support
  - `TMC5160MultiNode` helper class for multi-device management
- ✅ **Automatic Detection**: Chip version detection (TMC5130 vs TMC5160)

### ⚙️ Advanced Features

- ✅ **CoolStep**: Automatic current reduction when load is low for power efficiency
- ✅ **dcStep**: Automatic commutation for DC motor-like operation
- ✅ **Microstep Lookup Tables**: Custom microstep interpolation for optimized motion profiles
- ✅ **Unit Conversions**: Physical unit support (millimeters, degrees, RPM, belt teeth)
  - Lead screw systems (mm pitch)
  - Belt drive systems (belt pitch, pulley teeth)
  - Direct drive and gearbox systems
- ✅ **OTP Programming**: One-time programmable memory read/write support
- ✅ **Factory Configuration**: Read factory calibration and configuration data
- ✅ **Motor Setup from Specs**: High-level configuration from physical motor parameters

### 🏗️ Platform & Architecture

- ✅ **Hardware Agnostic**: SPI or UART interface for complete platform independence
- ✅ **CRTP Design**: Zero-overhead compile-time polymorphism
- ✅ **Template-Based**: Type-safe API with compile-time optimizations
- ✅ **Modern C++17**: RAII principles, constexpr functions, type safety
- ✅ **Complete Register Coverage**: All 47 registers (0x00-0x73) with 100+ public API methods
- ✅ **Subsystem Organization**: Logical grouping of functionality for easy discovery

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

### Class Structure & Subsystems

The `TMC5160` class is organized into intuitive subsystems for easy access to functionality:

```cpp
tmc5160::TMC5160<CommType> driver(comm_interface);

// Motion Control
driver.rampControl.SetRampMode(RampMode::POSITIONING);
driver.rampControl.SetTargetPosition(1000, Unit::Steps);
driver.rampControl.SetMaxSpeed(1000.0f, Unit::Steps);
driver.rampControl.SetAcceleration(500.0f, Unit::Steps);

// Motor Control
driver.motorControl.Enable();
driver.motorControl.SetCurrent(20, 10);  // IRUN, IHOLD
driver.motorControl.SetStealthChopEnabled(true);

// Diagnostics & Monitoring
DriverStatus status = driver.diagnostics.GetStatus();
uint16_t sg_value;
driver.diagnostics.GetStallGuard(sg_value);

// Automatic Tuning ⭐
StallGuardTuningResult result;
driver.tuning.TuneStallGuard(target_velocity, result, min_sgt, max_sgt, 
                              acceleration, min_velocity, max_velocity);

// Homing
int32_t final_position;
driver.homing.PerformSensorlessHoming(true, search_speed, final_position);

// Encoder Integration
driver.encoder.ConfigureEncoder(encoder_config);
driver.encoder.EnableDeviationDetection(100);  // 100 step threshold

// Protection
driver.protection.ConfigureShortProtection(short_config);
bool ot = driver.diagnostics.GetStatus() == DriverStatus::OT;
```

### Key Methods by Subsystem

#### RampControl Subsystem
| Method | Description |
|--------|-------------|
| `SetRampMode()` | Set mode: POSITIONING, VELOCITY_POS, VELOCITY_NEG, HOLD |
| `SetTargetPosition()` | Set target position (unit-aware: Steps, Mm, Deg, RPM) |
| `SetMaxSpeed()` | Set maximum velocity (unit-aware) |
| `SetAcceleration()` | Set acceleration/deceleration (unit-aware) |
| `IsTargetReached()` | Check if position target reached |
| `Stop()` | Stop motor immediately |
| `ConfigureReferenceSwitch()` | Configure endstops/reference switches |

#### MotorControl Subsystem
| Method | Description |
|--------|-------------|
| `Enable()` / `Disable()` | Enable/disable motor driver |
| `SetCurrent()` | Set run and hold currents |
| `SetStealthChopEnabled()` | Enable/disable silent stealthChop mode |
| `SetModeChangeSpeeds()` | Configure velocity thresholds for mode switching |

#### Diagnostics Subsystem
| Method | Description |
|--------|-------------|
| `GetStatus()` | Get comprehensive driver status |
| `GetStallGuard()` | Read StallGuard2 value (0-1023) |
| `ConfigureStallGuard()` | Configure StallGuard2 settings |
| `IsOpenLoadA()` / `IsOpenLoadB()` | Check for open load conditions |
| `VerifySetup()` | Run comprehensive startup verification |

#### Tuning Subsystem ⭐ NEW
| Method | Description |
|--------|-------------|
| `TuneStallGuard()` | Automatic SGT tuning with comprehensive velocity range analysis |
| Returns `StallGuardTuningResult` with optimal SGT, velocity compatibility, and actual achievable velocities |

#### Homing Subsystem
| Method | Description |
|--------|-------------|
| `PerformSensorlessHoming()` | Sensorless homing using StallGuard2 |
| `PerformSwitchHoming()` | Homing using reference switches |

#### Encoder Subsystem
| Method | Description |
|--------|-------------|
| `ConfigureEncoder()` | Configure encoder settings |
| `EnableDeviationDetection()` | Enable step loss detection |
| `GetDeviation()` | Read position deviation |

#### Protection Subsystem
| Method | Description |
|--------|-------------|
| `ConfigureShortProtection()` | Configure short circuit protection |
| `ConfigureOvertemperature()` | Configure overtemperature protection |

#### Communication Subsystem
| Method | Description |
|--------|-------------|
| `SetDaisyChainPosition()` | Set SPI daisy chain position |
| `SetDaisyChainLength()` | Configure SPI chain length |
| `ConfigureUartNodeAddress()` | Configure UART node address |

### Multi-Chip Support

| Class/Method | Description |
|--------------|-------------|
| `TMC5160DaisyChain<CommType, MaxDevices>` | High-level manager for SPI daisy chaining |
| `TMC5160MultiNode<CommType>` | High-level manager for UART multi-node addressing |
| `chain[0]`, `chain[1]`, ... | Access individual drivers in chain |

### Unit Conversion Helpers

| Function | Description |
|----------|-------------|
| `MmToSteps()` / `StepsToMm()` | Convert between millimeters and steps |
| `RpmToStepsPerSec()` / `StepsPerSecToRpm()` | Convert between RPM and steps/s |
| `DegreesToSteps()` / `StepsToDegrees()` | Convert between degrees and steps |

For complete API documentation with all methods and parameters, see [docs/api_reference.md](docs/api_reference.md).

## 📊 Examples

Comprehensive ESP32 examples are available in the [examples/esp32](examples/esp32/) directory, demonstrating all major features:

### Core Functionality
- **Basic Motor Control**: Simple positioning and velocity control
- **Ramp Control**: Advanced motion profiles with multi-phase acceleration
- **Motor Configuration**: Current control, chopper modes, stealthChop

### Multi-Chip Communication
- **`spi_daisy_chain_comprehensive_test.cpp`**: SPI daisy chaining with automatic chain detection
- **`uart_multi_node_comprehensive_test.cpp`**: UART multi-node addressing with sequential programming

### Advanced Features
- **`stallguard_tuning.cpp`**: Automatic StallGuard2 threshold tuning with velocity range analysis ⭐
- **`encoder_comprehensive_test.cpp`**: Encoder integration and closed-loop control with deviation detection
- **`diagnostics_comprehensive_test.cpp`**: Comprehensive diagnostics, status monitoring, and StallGuard2 reading
- **`protection_comprehensive_test.cpp`**: Protection system configuration and monitoring
- **`internal_ramp_sinusoidal.cpp`**: Sinusoidal motion profiles using internal ramp generator

### Motor Control & Motion
- **`motor_control_comprehensive_test.cpp`**: Advanced motor control features (CoolStep, dcStep, freewheeling)
- **`ramp_control_comprehensive_test.cpp`**: Ramp control modes and motion profiles
- **`bounds_finding_sinuous_motion.cpp`**: Advanced motion control with automatic bounds finding

### Specialized Applications
- **`internal_ramp_comprehensive_test.cpp`**: Internal ramp generator comprehensive testing
- **`stallguard_tuning.cpp`**: Automatic SGT tuning for optimal stall detection

All examples include detailed documentation and are ready to compile and run. See [docs/examples.md](docs/examples.md) for detailed walkthroughs.

## 📚 Documentation

For complete documentation, see the [docs directory](docs/index.md).

## 🤝 Contributing

Pull requests and suggestions are welcome! Please follow the existing code style and include tests for new features.

## 📄 License

This project is licensed under the **GNU General Public License v3.0**.
See the [LICENSE](LICENSE) file for details.

