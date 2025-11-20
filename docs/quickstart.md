---
layout: default
title: "⚡ Quick Start"
description: "Get up and running with the TMC5160 driver in minutes"
nav_order: 2
parent: "📚 Documentation"
permalink: /docs/quickstart/
---

# Quick Start

This guide will get you up and running with the TMC5160 driver in just a few steps.

## Prerequisites

- [Driver installed](installation.md)
- [Hardware wired](hardware_setup.md)
- Communication interface implemented (see [Platform Integration](platform_integration.md))

## Minimal Example

Here's a complete working example:

```cpp
#include "inc/tmc5160.hpp"

// 1. Implement your communication interface (see platform_integration.md)
class MySPI : public tmc5160::SpiCommInterface<MySPI> {
public:
    CommMode GetMode() const noexcept { return CommMode::SPI; }
    bool SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) {
        // Your SPI transfer implementation
        // CSN control is handled here (hardware SPI peripheral typically handles it automatically)
        // For daisy-chaining, ensure CSN stays low during entire transfer
        return true;
    }
    bool GpioSet(TMC5160CtrlPin pin, GpioSignal signal) {
        // Your GPIO set implementation
        return true;
    }
    bool GpioRead(TMC5160CtrlPin pin, GpioSignal& signal) {
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
    MySPI spi(true, true, true); // EN, DIR, STEP active high
    
    // 3. Create driver instance
    tmc5160::TMC5160 driver(spi);
    
    // 4. Initialize driver
    tmc5160::DriverConfig cfg{};
    cfg.motor.irun = 20;   // Run current (0-31)
    cfg.motor.ihold = 10;  // Hold current (0-31)
    
    if (!driver.Initialize(cfg)) {
        // Handle initialization failure
        return -1;
    }
    
    // 5. Configure ramp control
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(1000);  // 1000 steps
    driver.rampControl.SetMaxSpeed(1000.0f);     // 1000 steps/s
    driver.rampControl.SetAcceleration(500.0f);  // 500 steps/s²
    
    // 6. Enable motor
    driver.motorControl.Enable();
    
    // 7. Wait for target reached
    while (!driver.rampControl.IsTargetReached()) {
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
tmc5160::TMC5160 driver(spi);
```

The driver takes a reference to your communication interface.

### Step 3: Initialize Driver

```cpp
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;
cfg.motor.ihold = 10;
driver.Initialize(cfg);
```

Configure motor currents and other settings, then initialize the driver.

### Step 4: Configure Ramp Control

```cpp
driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
driver.rampControl.SetTargetPosition(1000);
driver.rampControl.SetMaxSpeed(1000.0f);
driver.rampControl.SetAcceleration(500.0f);
```

Set the ramp mode, target position, maximum speed, and acceleration.

### Step 5: Enable Motor

```cpp
driver.motorControl.Enable();
```

Enable the motor driver to start motion.

## Expected Output

When running this example, the motor should:
1. Move to position 1000 steps
2. Accelerate to 1000 steps/s
3. Decelerate and stop at the target position

## Troubleshooting

If you encounter issues:

- **Compilation errors**: Check that you've implemented all required communication interface methods
- **Motor doesn't move**: Verify hardware connections and motor enable pin
- **Communication errors**: Check SPI/UART wiring and clock frequency
- **See**: [Troubleshooting](troubleshooting.md) for common issues

## Multi-Chip Daisy-Chaining

For multiple TMC5160 drivers on a single SPI bus:

```cpp
// Create one SPI interface (shared by all chips)
MySPI spi(true, true, true);
spi.Initialize();

// Option 1: Auto-detect chain length
uint8_t chain_length = spi.AutoDetectChainLength(8); // Probe up to 8 devices
if (chain_length > 0) {
    // Chain length automatically set
}

// Option 2: Manually set chain length (if known)
spi.SetDaisyChainLength(3); // 3 devices in chain

// Create multiple drivers with different daisy-chain positions
tmc5160::TMC5160 driver1(spi, 12'000'000, 0); // Position 0 (first chip)
tmc5160::TMC5160 driver2(spi, 12'000'000, 1); // Position 1 (second chip)
tmc5160::TMC5160 driver3(spi, 12'000'000, 2); // Position 2 (third chip)

// Each driver automatically uses its own position
driver1.Initialize(cfg); // Accesses chip 0
driver2.Initialize(cfg); // Accesses chip 1
driver3.Initialize(cfg); // Accesses chip 2

// Or use TMC5160DaisyChain for easier management
tmc5160::TMC5160DaisyChain<MySPI, 5> chain(spi, 3, 12'000'000);
chain.InitializeAll(cfg);
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

