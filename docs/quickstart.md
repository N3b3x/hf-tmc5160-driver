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

## Next Steps

- Explore [Examples](examples.md) for more advanced usage
- Review the [API Reference](api_reference.md) for all available methods
- Check [Configuration](configuration.md) for customization options

---

**Navigation**
⬅️ [Installation](installation.md) | [Next: Hardware Setup ➡️](hardware_setup.md) | [Back to Index](index.md)

