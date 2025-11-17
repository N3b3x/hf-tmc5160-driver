---
layout: default
title: "💡 Examples"
description: "Complete example walkthroughs for the TMC5160 driver"
nav_order: 7
parent: "📚 Documentation"
permalink: /docs/examples/
---

# Examples

This guide provides complete, working examples demonstrating various use cases for the TMC5160 driver.

## Example 1: Basic Positioning Mode

This example shows basic stepper motor control in positioning mode.

```cpp
#include "inc/tmc5160.hpp"

// Assume MySPI is implemented (see platform_integration.md)
MySPI spi(true, true, true); // EN, DIR, STEP active high
tmc5160::TMC5160 driver(spi);

int main() {
    // Initialize driver
    tmc5160::DriverConfig cfg{};
    cfg.motor.irun = 20;
    cfg.motor.ihold = 10;
    driver.Initialize(cfg);
    
    // Configure positioning mode
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(1000);  // Move 1000 steps
    driver.rampControl.SetMaxSpeed(1000.0f);     // 1000 steps/s
    driver.rampControl.SetAcceleration(500.0f);   // 500 steps/s²
    
    // Enable motor
    driver.motorControl.Enable();
    
    // Wait for target reached
    while (!driver.rampControl.IsTargetReached()) {
        // Motion in progress
    }
    
    return 0;
}
```

### Explanation

1. **Initialize**: Configure motor currents and initialize driver
2. **Set Ramp Mode**: Select positioning mode
3. **Configure Motion**: Set target position, speed, and acceleration
4. **Enable**: Enable motor driver
5. **Wait**: Poll until target position is reached

## Example 2: Velocity Mode

This example demonstrates velocity mode operation.

```cpp
#include "inc/tmc5160.hpp"

MySPI spi(true, true, true);
tmc5160::TMC5160 driver(spi);

int main() {
    tmc5160::DriverConfig cfg{};
    cfg.motor.irun = 20;
    driver.Initialize(cfg);
    
    // Set velocity mode
    driver.rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
    driver.rampControl.SetMaxSpeed(500.0f);  // 500 steps/s forward
    driver.rampControl.SetAcceleration(200.0f);
    
    driver.motorControl.Enable();
    
    // Motor runs continuously at 500 steps/s
    // To stop, call driver.rampControl.Stop()
    
    return 0;
}
```

## Example 3: StealthChop Silent Operation

This example configures the driver for silent operation using stealthChop.

```cpp
#include "inc/tmc5160.hpp"

MySPI spi(true, true, true);
tmc5160::TMC5160 driver(spi);

int main() {
    tmc5160::DriverConfig cfg{};
    cfg.motor.irun = 20;
    cfg.motor.ihold = 10;
    cfg.chopper.mres = 4;  // 16 microsteps for smooth operation
    cfg.stealthchop.pwm_autoscale = true;
    cfg.stealthchop.pwm_autograd = true;
    driver.Initialize(cfg);
    
    // Configure stealthChop thresholds
    // Below 100 steps/s: stealthChop mode (silent)
    // Above 100 steps/s: spreadCycle mode (more torque)
    driver.motorControl.SetModeChangeSpeeds(100.0f, 0.0f, 0.0f);
    
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(1000);
    driver.rampControl.SetMaxSpeed(50.0f);  // Low speed = stealthChop
    
    driver.motorControl.Enable();
    
    while (!driver.rampControl.IsTargetReached()) {
        // Silent operation
    }
    
    return 0;
}
```

## Example 4: Encoder Closed-Loop Control

This example demonstrates encoder-based closed-loop control.

```cpp
#include "inc/tmc5160.hpp"

MySPI spi(true, true, true);
tmc5160::TMC5160 driver(spi);

int main() {
    tmc5160::DriverConfig cfg{};
    cfg.motor.irun = 20;
    driver.Initialize(cfg);
    
    // Configure encoder
    tmc5160::EncoderConfig enc_cfg{};
    enc_cfg.enc_sel_decimal = false; // Binary mode
    driver.encoder.Configure(enc_cfg);
    
    // Set encoder resolution: 200 steps/rev motor, 1000 pulses/rev encoder
    driver.encoder.SetResolution(200, 1000, false);
    driver.encoder.SetAllowedDeviation(10); // 10 steps tolerance
    
    // Move to position
    driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);
    driver.rampControl.SetTargetPosition(1000);
    driver.rampControl.SetMaxSpeed(1000.0f);
    driver.motorControl.Enable();
    
    // Monitor encoder deviation
    while (!driver.rampControl.IsTargetReached()) {
        if (driver.encoder.IsDeviationDetected()) {
            // Step loss detected - take corrective action
            driver.encoder.ClearDeviationFlag();
        }
    }
    
    return 0;
}
```

## Example 5: StallGuard Stall Detection

This example shows how to use StallGuard2 for stall detection.

```cpp
#include "inc/tmc5160.hpp"

MySPI spi(true, true, true);
tmc5160::TMC5160 driver(spi);

int main() {
    tmc5160::DriverConfig cfg{};
    cfg.motor.irun = 20;
    driver.Initialize(cfg);
    
    // Configure StallGuard2
    tmc5160::StallGuardConfig sg_cfg{};
    sg_cfg.sgt = 0;      // Threshold (tune for your motor)
    sg_cfg.semin = 0;    // Minimum SG value
    sg_cfg.semax = 0;    // Hysteresis
    sg_cfg.sfilt = false; // Filter disabled
    driver.diagnostics.ConfigureStallGuard(sg_cfg);
    
    driver.rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_POS);
    driver.rampControl.SetMaxSpeed(500.0f);
    driver.motorControl.Enable();
    
    // Monitor StallGuard value
    while (true) {
        uint16_t sg_value = driver.diagnostics.GetStallGuard();
        if (sg_value < 100) {  // Threshold depends on motor
            // Stall detected - stop motor
            driver.rampControl.Stop();
            break;
        }
    }
    
    return 0;
}
```

## Running the Examples

### ESP32

```bash
cd examples/esp32
idf.py build flash monitor
```

### Other Platforms

Compile with C++17 support:
```bash
g++ -std=c++17 -I inc/ example.cpp -o example
```

## Next Steps

- Review the [API Reference](api_reference.md) for method details
- Check [Troubleshooting](troubleshooting.md) if you encounter issues
- Explore the [examples directory](../examples/) for more examples

---

**Navigation**
⬅️ [API Reference](api_reference.md) | [Next: Troubleshooting ➡️](troubleshooting.md) | [Back to Index](index.md)

