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

## Example 6: Bounds Finding and Sinuous Motion

This comprehensive example demonstrates:
- **Sensorless bounds finding** using StallGuard2 in both directions
- **Bounded and unbounded modes** (handles cases with/without mechanical stops)
- **Degree/radian support** for intuitive angle-based control
- **Home position reset** by relative angles
- **Sinuous motion** between bounds with configurable wait times and waypoints

### Key Features

1. **Automatic Bounds Detection**: Finds mechanical stops in both directions using sensorless homing
2. **Unbounded Mode**: Handles cases where no stops are found, using current position as home
3. **Angle-Based Control**: Work with degrees or radians instead of steps
4. **Home Reset**: Reset home position by relative angle offset
5. **Sinuous Motion**: Smooth sinusoidal motion between bounds with customizable parameters

### Basic Usage

```cpp
#include "inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"

// Create driver instance
Esp32SPI spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
             GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15, 4000000);
tmc5160::TMC5160 driver(spi);

// Initialize driver
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;
cfg.motor.ihold = 10;
cfg.chopper.mres = 5; // 32 microsteps
driver.Initialize(cfg);

// Create motion controller
BoundedSinuousMotion motion(&driver);

// Configure motor parameters (needed for degree/radian conversions)
uint16_t steps_per_rev = 200 * 32; // 200 steps * 32 microsteps
motion.ConfigureMotor(steps_per_rev, AngleUnit::DEGREES);

// After finding bounds (see full example), set bounds in degrees
motion.SetBoundsDegrees(-90.0f, 90.0f);  // -90° to +90° from home

// Or set bounds in radians
motion.SetBoundsRadians(-M_PI/2, M_PI/2);

// Reset home position by relative degrees
motion.ResetHomeByDegrees(45.0f);  // Move home +45° from current position

// Configure sinuous motion
motion.SetSinuousAmplitudeDegrees(60.0f);  // 60° amplitude
motion.SetSinuousParams(0, 0.5f);  // 0.5 Hz frequency

// Set wait times at bounds (can be 0 to disable)
motion.SetDefaultWaits(500, 500, 300);  // min, max, home (ms)

// Add waypoints (optional)
motion.AddWaypoint(tmc5160::DegreesToSteps(-30.0f, steps_per_rev), 200);
motion.AddWaypoint(tmc5160::DegreesToSteps(30.0f, steps_per_rev), 200);

// Start motion
motion.Start();

// Update in main loop
while (true) {
    motion.Update();
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

### Unbounded Mode

When no mechanical stops are detected, the system automatically enters unbounded mode:

```cpp
// System detects no stops found
if (!motion.IsBounded()) {
    // Use current position as home
    motion.SetUnbounded(current_position, 10000);  // Default range: 10000 steps
    
    // User can reset home to any relative angle
    motion.ResetHomeByDegrees(45.0f);
    
    // Bounds are automatically recalculated
    float min_deg, max_deg;
    motion.GetBoundsDegrees(min_deg, max_deg);
}
```

### Working with Angles

The example provides full support for degree and radian operations:

```cpp
// Set bounds in degrees (relative to home)
motion.SetBoundsDegrees(-90.0f, 90.0f);

// Set bounds in radians
motion.SetBoundsRadians(-M_PI/2, M_PI/2);

// Get bounds in degrees
float min_deg, max_deg;
motion.GetBoundsDegrees(min_deg, max_deg);

// Get bounds in radians
float min_rad, max_rad;
motion.GetBoundsRadians(min_rad, max_rad);

// Reset home by degrees
motion.ResetHomeByDegrees(30.0f);

// Reset home by radians
motion.ResetHomeByRadians(M_PI/6);

// Set sinuous amplitude in degrees
motion.SetSinuousAmplitudeDegrees(45.0f);

// Set sinuous amplitude in radians
motion.SetSinuousAmplitudeRadians(M_PI/4);
```

### Waypoint Management

Add, remove, and manage waypoints with wait times:

```cpp
// Add waypoints
motion.AddWaypoint(position_in_steps, wait_time_ms);

// Remove waypoint by index
motion.RemoveWaypoint(0);

// Clear all waypoints
motion.ClearWaypoints();

// Get waypoint count
size_t count = motion.GetWaypointCount();
```

### Complete Example Flow

1. **Find Bounds**: Automatically detects mechanical stops in both directions
2. **Set Home**: Sets middle position as home (or uses current position if unbounded)
3. **Configure**: Set up sinuous motion parameters, wait times, and waypoints
4. **Start Motion**: Begin sinuous motion pattern
5. **Update Loop**: Continuously update motion controller

### Running the Example

For ESP32:

```bash
cd examples/esp32
idf.py set-target esp32c6  # or your target
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Select the example:
```bash
idf.py menuconfig
# Navigate to: Example Configuration -> Example to run
# Select: bounds_finding_sinuous_motion
```

Or build directly:
```bash
idf.py build -DAPP_TYPE=bounds_finding_sinuous_motion
```

### Configuration Notes

- **Steps per Revolution**: Must be configured correctly for degree/radian conversions
  - Base steps: 200 for 1.8° motors, 400 for 0.9° motors
  - With microsteps: multiply by microstep factor (e.g., 200 × 32 = 6400)
- **Stall Threshold**: Tune `sgt` parameter for your motor and mechanical system
- **Search Speed**: Adjust based on your application (typically 200-1000 steps/s)
- **Default Range**: For unbounded mode, set appropriate default range based on your application

### See Also

- [Sensorless Homing Guide](special_features_sensorless_homing.md) - Detailed StallGuard2 configuration
- [Unit Conversions](special_features_unit_conversions.md) - Physical unit conversion functions
- [API Reference](api_reference.md) - Complete API documentation

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

