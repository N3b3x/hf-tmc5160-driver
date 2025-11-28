---
layout: default
title: "⚡ Quick Start"
description: "Get up and running with the TMC51x0 driver (TMC5130 & TMC5160) in minutes"
nav_order: 2
parent: "📚 Documentation"
permalink: /docs/quickstart/
---

# Quick Start

This guide will get you up and running with the TMC51x0 driver (TMC5130 & TMC5160) in just a few steps.

## Prerequisites

- [Driver installed](installation.md)
- [Hardware wired](hardware_setup.md)
- Communication interface implemented (see [Platform Integration](platform_integration.md))

## Class Structure Overview

The TMC51x0 driver uses a **subsystem-based architecture** that organizes functionality into logical groups:

```cpp
tmc51x0::TMC51x0<MySPI> driver(spi);

// Organized subsystems for intuitive access
driver.rampControl      // Motion planning, positioning, velocity control
driver.motorControl     // Current control, chopper modes, stealthChop
driver.encoder          // Encoder integration, closed-loop control
driver.diagnostics      // Status monitoring, StallGuard2, diagnostics
driver.tuning           // Automatic parameter optimization (SGT tuning) ⭐
driver.homing           // Sensorless and switch-based homing
driver.protection       // Safety systems, short circuit protection
driver.communication    // Multi-chip communication setup
driver.printer          // Debug register printing
```

Each subsystem provides focused methods for a specific aspect of motor control, making it easy to discover and use features.

## Minimal Example

Here's a complete working example:

```cpp
#include "inc/tmc51x0.hpp"

// 1. Implement your communication interface (see platform_integration.md)
class MySPI : public tmc51x0::SpiCommInterface<MySPI> {
public:
    CommMode GetMode() const noexcept { return CommMode::SPI; }
    bool SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) {
        // Your SPI transfer implementation
        // CSN control is handled here (hardware SPI peripheral typically handles it automatically)
        // For daisy-chaining, ensure CSN stays low during entire transfer
        return true;
    }
    bool GpioSet(TMC51x0CtrlPin pin, GpioSignal signal) {
        // Your GPIO set implementation
        return true;
    }
    bool GpioRead(TMC51x0CtrlPin pin, GpioSignal& signal) {
        // Your GPIO read implementation
        return true;
    }
    void DebugLog(int level, const char* tag, const char* format, va_list args) {
        // Your logging implementation (optional)
    }
    void DelayMs(uint32_t ms) {
        // Your delay implementation
    }
    void DelayUs(uint32_t us) {
        // Your delay implementation
    }
};

int main() {
    // 2. Create communication interface
    MySPI spi;
    spi.Initialize(); // Initialize your SPI hardware
    
    // 3. Create driver instance
    tmc51x0::TMC51x0<MySPI> driver(spi);
    
    // 4. Initialize driver
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.rated_current_ma = 2000;  // 2A rated current
    cfg.motor_spec.sense_resistor_mohm = 50;  // 0.05Ω sense resistor
    cfg.motor_spec.supply_voltage_mv = 24000; // 24V supply
    // IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated - DO NOT set manually
    
    if (!driver.Initialize(cfg)) {
        // Handle initialization failure
        return -1;
    }
    
    // 5. Configure ramp control (RampControl subsystem)
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(1000);  // 1000 steps
    // Velocity functions default to revolutions per second (RevPerSec)
    driver.rampControl.SetMaxSpeed(0.02f);       // 0.02 rev/s (~1.2 RPM) - Unit::RevPerSec is default
    driver.rampControl.SetAcceleration(0.01f);    // 0.01 rev/s² - Unit::RevPerSec is default
    
    // 6. Enable motor (MotorControl subsystem)
    driver.motorControl.Enable();
    
    // 7. Wait for target reached
    while (!driver.rampControl.IsTargetReached()) {
        // Optional: Monitor status (Diagnostics subsystem)
        tmc51x0::DriverStatus status = driver.diagnostics.GetStatus();
        if (status != tmc51x0::DriverStatus::OK) {
            // Handle error condition
            break;
        }
        // Wait for motion to complete
    }
    
    return 0;
}
```

## Step-by-Step Explanation

### Step 1: Implement Communication Interface

You must implement either `SpiCommInterface` or `UartCommInterface` for your platform. See
[Platform Integration](platform_integration.md) for detailed examples.

### Step 2: Create Driver Instance

```cpp
tmc51x0::TMC51x0 driver(spi);
```

The driver takes a reference to your communication interface.

### Step 3: Initialize Driver

```cpp
tmc51x0::DriverConfig cfg{};
cfg.motor_spec.rated_current_ma = 2000;  // 2A rated current
cfg.motor_spec.sense_resistor_mohm = 50;  // 0.05Ω sense resistor
cfg.motor_spec.supply_voltage_mv = 24000; // 24V supply
// IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated - DO NOT set manually
driver.Initialize(cfg);
```

Configure motor specifications and other settings, then initialize the driver. The driver automatically calculates IRUN, IHOLD, and GLOBAL_SCALER from your motor specifications.

### Step 4: Configure Ramp Control

```cpp
driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
driver.rampControl.SetTargetPosition(1000);
// Velocity functions default to revolutions per second (RevPerSec)
driver.rampControl.SetMaxSpeed(0.02f);       // 0.02 rev/s (~1.2 RPM)
driver.rampControl.SetAcceleration(0.01f);  // 0.01 rev/s²
```

Set the ramp mode, target position, maximum speed, and acceleration. Note that velocity-related functions now default to `Unit::RevPerSec` (revolutions per second), making it easier to work with intuitive units.

### Step 5: Enable Motor

```cpp
driver.motorControl.Enable();
```

Enable the motor driver to start motion.

## Expected Output

When running this example, the motor should:
1. Move to position 1000 steps
2. Accelerate to 0.02 rev/s (~1.2 RPM)
3. Decelerate and stop at the target position

## Exploring Subsystems

Once you have basic motion working, explore the other subsystems:

### Diagnostics & Monitoring
```cpp
// Check driver status
tmc51x0::DriverStatus status = driver.diagnostics.GetStatus();

// Read StallGuard2 value (load measurement)
uint16_t sg_value;
driver.diagnostics.GetStallGuard(sg_value);

// Verify setup
driver.diagnostics.VerifySetup();
```

### Automatic Tuning ⭐
```cpp
// Automatically find optimal StallGuard2 threshold
tmc51x0::StallGuardTuningResult result;
// Using AutoTuneStallGuard (recommended) - includes safe current margin
// Note: Separate unit parameters for velocity and acceleration
driver.tuning.AutoTuneStallGuard(
    0.6f, result,                          // Target velocity: 0.6 rev/s (~36 RPM)
    0, 63,                                 // SGT search range
    0.06f,                                 // Acceleration: 0.06 rev/s²
    0.18f, 0.9f,                           // Velocity range: 0.18-0.9 rev/s (30%-150% of target)
    tmc51x0::Unit::RevPerSec,              // Velocity unit (default, can be omitted)
    tmc51x0::Unit::RevPerSec,              // Acceleration unit (default, can be omitted)
    300                        // Safe current margin: 300mA
);

// Use the optimal SGT value
tmc51x0::StallGuardConfig sg_config;
sg_config.threshold = result.optimal_sgt;
driver.diagnostics.ConfigureStallGuard(sg_config);
```

### Sensorless Homing
```cpp
// Home without endstops using StallGuard2
int32_t final_position;
driver.homing.PerformSensorlessHoming(
    true,           // Direction (true = positive)
    search_speed,   // Search speed
    final_position  // Final position after homing
);
```

### Encoder Integration
```cpp
// Configure encoder for closed-loop control
tmc51x0::EncoderConfig enc_cfg{};
enc_cfg.resolution = 4096;  // Encoder resolution
driver.encoder.ConfigureEncoder(enc_cfg);

// Enable step loss detection
driver.encoder.EnableDeviationDetection(100);  // 100 step threshold
```

## Troubleshooting

If you encounter issues:

- **Compilation errors**: Check that you've implemented all required communication interface methods
- **Motor doesn't move**: Verify hardware connections and motor enable pin
- **Communication errors**: Check SPI/UART wiring and clock frequency
- **See**: [Troubleshooting](troubleshooting.md) for common issues

## Multi-Chip Daisy-Chaining

For multiple TMC51x0 drivers on a single SPI bus:

```cpp
// Create one SPI interface (shared by all chips)
MySPI spi;
spi.Initialize();

// Option 1: Use TMC51x0DaisyChain helper class (recommended)
tmc51x0::TMC51x0DaisyChain<MySPI, 5> chain(spi, 3);
// Auto-detects chain length and configures properly
chain.InitializeAll(cfg);

// Access individual drivers
auto& driver1 = chain[0]; // Position 0 (first chip)
auto& driver2 = chain[1]; // Position 1 (second chip)
auto& driver3 = chain[2]; // Position 2 (third chip)

// Option 2: Manual setup
// Auto-detect chain length
uint8_t chain_length = spi.AutoDetectChainLength(8); // Probe up to 8 devices
if (chain_length > 0) {
    spi.SetDaisyChainLength(chain_length);
}

// Create multiple drivers with different daisy-chain positions
tmc51x0::TMC51x0<MySPI> driver1(spi, 0); // Position 0 (first chip)
tmc51x0::TMC51x0<MySPI> driver2(spi, 1); // Position 1 (second chip)
tmc51x0::TMC51x0<MySPI> driver3(spi, 2); // Position 2 (third chip)

// Each driver automatically uses its own position
driver1.Initialize(cfg); // Accesses chip 0
driver2.Initialize(cfg); // Accesses chip 1
driver3.Initialize(cfg); // Accesses chip 2
```

See [Multi-Chip Communication](special_features_multi_chip.md) for detailed information.

## Next Steps

- Explore [Examples](examples.md) for more advanced usage
- Review the [API Reference](api_reference.md) for all available methods
- Check [Configuration](configuration.md) for customization options
- Learn about [Multi-Chip Communication](special_features_multi_chip.md) for daisy-chaining

---

**Navigation**
⬅️ [Installation](installation.md) | [Next: Hardware Setup ➡️](hardware_setup.md) | [Back to Index](index.md)

