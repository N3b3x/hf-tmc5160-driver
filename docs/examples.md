---
layout: default
title: "💡 Examples"
description: "Complete example walkthroughs for the TMC51x0 driver (TMC5130 & TMC5160)"
nav_order: 7
parent: "📚 Documentation"
permalink: /docs/examples/
---

# Examples

This guide provides complete, working examples demonstrating various use cases for the TMC51x0 driver (TMC5130 & TMC5160).

## Example 1: Basic Positioning Mode

This example shows basic stepper motor control in positioning mode.

```cpp
#include "inc/tmc51x0.hpp"

// Assume MySPI is implemented (see platform_integration.md)
MySPI spi(true, true, true); // EN, DIR, STEP active high
tmc51x0::TMC51x0 driver (TMC5130 & TMC5160)(spi);

int main() {
    // Initialize driver
    tmc51x0::DriverConfig cfg{};
    // Motor current is automatically calculated from motor_spec
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;  // Required for calculation
    cfg.motor_spec.supply_voltage_mv = 24000;  // Required for calculation
    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Initialization error: %s\n", init_result.ErrorMessage());
        return -1;
    }
    
    // Configure positioning mode
    auto mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    if (!mode_result) {
        printf("Error setting ramp mode: %s\n", mode_result.ErrorMessage());
        return -1;
    }
    
    auto pos_result = driver.rampControl.SetTargetPosition(1000.0f, tmc51x0::Unit::Steps);  // Move 1000 steps
    if (!pos_result) {
        printf("Error setting target position: %s\n", pos_result.ErrorMessage());
        return -1;
    }
    // Velocity defaults to revolutions per second - 0.02 rev/s ≈ 1.2 RPM for typical motor
    driver.rampControl.SetMaxSpeed(0.02f);     // Unit::RevPerSec is default
    driver.rampControl.SetAcceleration(0.01f); // Unit::RevPerSec is default for acceleration too
    
    // Enable motor
    auto enable_result = driver.motorControl.Enable();
    if (!enable_result) {
        printf("Error enabling motor: %s\n", enable_result.ErrorMessage());
        return -1;
    }
    
    // Wait for target reached
    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached && reached.Value()) {
            break; // Target reached
        }
        if (!reached) {
            printf("Error checking target: %s\n", reached.ErrorMessage());
            break;
        }
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
#include "inc/tmc51x0.hpp"

MySPI spi(true, true, true);
tmc51x0::TMC51x0 driver (TMC5130 & TMC5160)(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    // Motor current is automatically calculated from motor_spec
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Initialization error: %s\n", init_result.ErrorMessage());
        return -1;
    }
    
    // Set velocity mode
    auto mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_POS);
    if (!mode_result) {
        printf("Error setting ramp mode: %s\n", mode_result.ErrorMessage());
        return -1;
    }
    // Using revolutions per second (default) - 0.01 rev/s ≈ 0.6 RPM for typical motor
    driver.rampControl.SetMaxSpeed(0.01f);  // Unit::RevPerSec is default
    driver.rampControl.SetAcceleration(0.005f);  // Unit::RevPerSec is default
    
    auto enable_result = driver.motorControl.Enable();
    if (!enable_result) {
        printf("Error enabling motor: %s\n", enable_result.ErrorMessage());
        return -1;
    }
    
    // Motor runs continuously at 0.01 rev/s
    // To stop, call driver.rampControl.Stop()
    
    return 0;
}
```

## Example 3: StealthChop Silent Operation

This example configures the driver for silent operation using stealthChop.

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi(true, true, true);
tmc51x0::TMC51x0 driver (TMC5130 & TMC5160)(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    // Motor current is automatically calculated from motor_spec
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    cfg.chopper.mres = tmc51x0::MicrostepResolution::MRES_256;  // 256 microsteps for smooth operation
    cfg.stealthchop.pwm_autoscale = true;
    cfg.stealthchop.pwm_autograd = true;
    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Initialization error: %s\n", init_result.ErrorMessage());
        return -1;
    }
    
    // Configure stealthChop thresholds
    // Below 0.002 rev/s (~0.12 RPM): stealthChop mode (silent)
    // Above 0.002 rev/s: spreadCycle mode (more torque)
    driver.thresholds.SetModeChangeSpeeds(0.002f, 0.0f, 0.0f, tmc51x0::Unit::RevPerSec);  // Unit::RevPerSec is default
    
    auto mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    if (!mode_result) {
        printf("Error setting ramp mode: %s\n", mode_result.ErrorMessage());
        return -1;
    }
    
    auto pos_result = driver.rampControl.SetTargetPosition(1000.0f, tmc51x0::Unit::Steps);
    if (!pos_result) {
        printf("Error setting target position: %s\n", pos_result.ErrorMessage());
        return -1;
    }
    driver.rampControl.SetMaxSpeed(0.001f);  // Low speed = stealthChop (Unit::RevPerSec is default)
    
    auto enable_result = driver.motorControl.Enable();
    if (!enable_result) {
        printf("Error enabling motor: %s\n", enable_result.ErrorMessage());
        return -1;
    }
    
    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached && reached.Value()) {
            break; // Target reached
        }
        if (!reached) {
            printf("Error checking target: %s\n", reached.ErrorMessage());
            break;
        }
        // Silent operation
    }
    
    return 0;
}
```

## Example 4: Encoder Closed-Loop Control

This example demonstrates encoder-based closed-loop control.

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi(true, true, true);
tmc51x0::TMC51x0 driver (TMC5130 & TMC5160)(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    // Motor current is automatically calculated from motor_spec
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Initialization error: %s\n", init_result.ErrorMessage());
        return -1;
    }
    
    // Configure encoder
    tmc51x0::EncoderConfig enc_cfg{};
    enc_cfg.prescaler_mode = tmc51x0::EncoderPrescalerMode::BINARY; // Binary mode
    auto enc_result = driver.encoder.Configure(enc_cfg);
    if (!enc_result) {
        printf("Error configuring encoder: %s\n", enc_result.ErrorMessage());
        return -1;
    }
    
    // Set encoder resolution: 200 steps/rev motor, 1000 pulses/rev encoder
    driver.encoder.SetResolution(200, 1000, false);
    driver.encoder.SetAllowedDeviation(10); // 10 steps tolerance
    
    // Move to position
    auto mode_result2 = driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    if (!mode_result2) {
        printf("Error setting ramp mode: %s\n", mode_result2.ErrorMessage());
        return -1;
    }
    
    auto pos_result2 = driver.rampControl.SetTargetPosition(1000.0f, tmc51x0::Unit::Steps);
    if (!pos_result2) {
        printf("Error setting target position: %s\n", pos_result2.ErrorMessage());
        return -1;
    }
    driver.rampControl.SetMaxSpeed(0.02f);  // Unit::RevPerSec is default
    auto enable_result2 = driver.motorControl.Enable();
    if (!enable_result2) {
        printf("Error enabling motor: %s\n", enable_result2.ErrorMessage());
        return -1;
    }
    
    // Monitor encoder deviation
    while (true) {
        auto reached2 = driver.rampControl.IsTargetReached();
        if (reached2 && reached2.Value()) {
            break; // Target reached
        }
        if (!reached2) {
            printf("Error checking target: %s\n", reached2.ErrorMessage());
            break;
        }
        
        auto dev_result = driver.encoder.IsDeviationWarning();
        if (dev_result && dev_result.Value()) {
            // Step loss detected - take corrective action
            driver.encoder.ClearDeviationWarning();
        }
    }
    
    return 0;
}
```

## Example 5: StallGuard Stall Detection

This example shows how to use StallGuard2 for stall detection.

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi(true, true, true);
tmc51x0::TMC51x0 driver (TMC5130 & TMC5160)(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    // Motor current is automatically calculated from motor_spec
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    auto init_result2 = driver.Initialize(cfg);
    if (!init_result2) {
        printf("Initialization error: %s\n", init_result2.ErrorMessage());
        return -1;
    }
    
    // Configure StallGuard2
    tmc51x0::StallGuardConfig sg_cfg{};
    sg_cfg.threshold = 0;      // Threshold (tune for your motor)
    sg_cfg.enable_filter = false; // Filter disabled
    // Note: semin/semax are CoolStep parameters, configure separately if needed
    auto sg_result = driver.stallGuard.ConfigureStallGuard(sg_cfg);
    if (!sg_result) {
        printf("Error configuring StallGuard: %s\n", sg_result.ErrorMessage());
        return -1;
    }
    
    auto mode_result3 = driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_POS);
    if (!mode_result3) {
        printf("Error setting ramp mode: %s\n", mode_result3.ErrorMessage());
        return -1;
    }
    driver.rampControl.SetMaxSpeed(0.01f);  // Unit::RevPerSec is default
    auto enable_result3 = driver.motorControl.Enable();
    if (!enable_result3) {
        printf("Error enabling motor: %s\n", enable_result3.ErrorMessage());
        return -1;
    }
    
    // Monitor StallGuard value
    while (true) {
        auto sg_result2 = driver.stallGuard.GetStallGuard();
        if (sg_result2) {
            uint16_t sg_value = sg_result2.Value();
            if (sg_value < 100) {  // Threshold depends on motor
                // Stall detected - stop motor
                driver.rampControl.Stop();
                break;
            }
        } else {
            printf("Error reading StallGuard: %s\n", sg_result2.ErrorMessage());
            break;
        }
    }
    
    return 0;
}
```

## Example 6: Fatigue Testing - Bounds Finding and Sinuous Motion

This example is designed for **cable/strain relief fatigue testing** and demonstrates:
- **Sensorless bounds finding** using StallGuard2 in both directions
- **Global bounds** (hardware limits) and **local bounds** (oscillation range)
- **Automatic clipping** of local bounds to global bounds
- **Degree/radian support** for intuitive angle-based control
- **Pure sinusoidal back-and-forth motion** optimized for fatigue testing
- **Dwell times** at bounds and optionally at center

### Key Features

1. **Global vs Local Bounds**: 
   - Global bounds are hardware limits found during initialization
   - Local bounds define the oscillation range for testing
   - Local bounds are automatically clipped to global bounds if they exceed them

2. **Fatigue Testing Optimized**: 
   - Pure sinusoidal motion between two bounds (ideal for fatigue testing)
   - Configurable dwell times at extremes (simulates holding tool in awkward positions)
   - Optional center dwell for specific test requirements

3. **Unbounded Mode**: Handles cases where no stops are found, using current position as home

4. **Angle-Based Control**: Work with degrees or radians instead of steps

### Basic Usage

```cpp
#include "inc/tmc51x0.hpp"
#include "esp32_tmc5160_bus.hpp"

// Create driver instance
Esp32SPI spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
             GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_15, 4000000);
tmc51x0::TMC51x0 driver (TMC5130 & TMC5160)(spi);

// Initialize driver
tmc51x0::DriverConfig cfg{};
// Motor current is automatically calculated from motor_spec
cfg.motor_spec.rated_current_ma = 1500;
cfg.motor_spec.sense_resistor_mohm = 50;
cfg.motor_spec.supply_voltage_mv = 24000;
cfg.chopper.mres = tmc51x0::MicrostepResolution::MRES_256; // 256 microsteps
auto init_result4 = driver.Initialize(cfg);
if (!init_result4) {
    printf("Initialization error: %s\n", init_result4.ErrorMessage());
    return -1;
}

// Create fatigue test motion controller
FatigueTestMotion motion(&driver);

// Configure motor parameters (needed for degree/radian conversions)
uint16_t steps_per_rev = 200 * 32; // 200 steps * 32 microsteps
motion.ConfigureMotor(steps_per_rev, AngleUnit::DEGREES);

// After finding global bounds (see full example), set global bounds
motion.SetGlobalBoundsDegrees(-90.0f, 90.0f);  // Hardware limits: -90° to +90°

// Set local bounds for oscillation (will be clipped to global bounds if needed)
motion.SetLocalBoundsDegrees(-60.0f, 60.0f);  // Oscillate ±60° from home

// Configure sinuous motion
motion.SetSinuousAmplitudeDegrees(60.0f);  // 60° amplitude
motion.SetSinuousParams(0, 0.5f);  // 0.5 Hz frequency

// Set dwell times at bounds (can be 0 to disable)
// For fatigue testing: dwell at extremes simulates holding tool in awkward positions
motion.SetDwellTimes(2000,  // 2 seconds at minimum bound
                      2000,  // 2 seconds at maximum bound
                      0);    // No dwell at center (set to >0 to enable)

// Set target cycle count (0 = infinite)
motion.SetTargetCycles(1000);  // Run for 1000 cycles, then auto-stop

// Start motion (can be called at any time)
motion.Start();

// Update in main loop
while (true) {
    motion.Update();
    
    // Settings can be changed in real-time while running:
    // motion.SetFrequency(1.0f);  // Change frequency
    // motion.SetSinuousAmplitudeDegrees(45.0f);  // Change amplitude
    // motion.SetDwellTimes(1000, 1000, 0);  // Change dwell times
    // motion.SetTargetCycles(2000);  // Change target cycles
    
    // Stop and restart at any time:
    // motion.Stop();
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // motion.Start();  // Resume from current position
    
    // Check status:
    // uint32_t cycles = motion.GetCurrentCycles();
    // bool running = motion.IsRunning();
    // bool complete = motion.IsCycleComplete();
    
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

### Global vs Local Bounds

The system distinguishes between global bounds (hardware limits) and local bounds (oscillation range):

```cpp
// Set global bounds (hardware limits found during initialization)
motion.SetGlobalBoundsDegrees(-90.0f, 90.0f);

// Set local bounds for oscillation (will be clipped to global bounds automatically)
motion.SetLocalBoundsDegrees(-60.0f, 60.0f);  // Oscillate ±60°

// If local bounds exceed global bounds, they are automatically clipped
motion.SetLocalBoundsDegrees(-100.0f, 100.0f);  // Will be clipped to ±90°
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
    
    // Get local bounds
    float min_deg, max_deg;
    motion.GetLocalBoundsDegrees(min_deg, max_deg);
}
```

### Working with Angles

The example provides full support for degree and radian operations:

```cpp
// Set global bounds in degrees
motion.SetGlobalBoundsDegrees(-90.0f, 90.0f);

// Set local bounds in degrees
motion.SetLocalBoundsDegrees(-60.0f, 60.0f);

// Set bounds in radians
motion.SetGlobalBoundsRadians(-M_PI/2, M_PI/2);
motion.SetLocalBoundsRadians(-M_PI/3, M_PI/3);

// Get bounds in degrees
float min_deg, max_deg;
motion.GetLocalBoundsDegrees(min_deg, max_deg);
motion.GetGlobalBoundsDegrees(min_deg, max_deg);

// Reset home by degrees
motion.ResetHomeByDegrees(30.0f);

// Reset home by radians
motion.ResetHomeByRadians(M_PI/6);

// Set sinuous amplitude in degrees
motion.SetSinuousAmplitudeDegrees(45.0f);

// Set sinuous amplitude in radians
motion.SetSinuousAmplitudeRadians(M_PI/4);
```

### Dwell Times

Configure dwell times at bounds and optionally at center (can be changed in real-time):

```cpp
// Set dwell times (in milliseconds, 0 to disable)
motion.SetDwellTimes(
    2000,  // Dwell at minimum bound: 2 seconds
    2000,  // Dwell at maximum bound: 2 seconds
    500    // Dwell at center: 0.5 seconds (optional, 0 to disable)
);

// For pure fatigue testing without dwells:
motion.SetDwellTimes(0, 0, 0);  // No dwells, continuous motion

// Change dwell times while running:
motion.SetDwellTimes(1000, 1000, 0);  // Update in real-time
```

### Cycle Count

Set target cycle count and track progress. **One cycle = center → min → max → center** (or center → max → min → center). Cycles are counted at the center crossing point (0 crossing).

```cpp
// Set target cycle count (0 = infinite)
motion.SetTargetCycles(1000);  // Run for 1000 cycles

// Get current cycle count
uint32_t cycles = motion.GetCurrentCycles();

// Get target cycles
uint32_t target = motion.GetTargetCycles();

// Check if cycle count reached
if (motion.IsCycleComplete()) {
    ESP_LOGI(TAG, "Test complete: %lu cycles", motion.GetCurrentCycles());
    // Motion automatically stops at center position when cycles complete
}

// Reset cycle count
motion.ResetCycles();

// Change target cycles while running
motion.SetTargetCycles(2000);  // Update target in real-time
```

**Note**: When target cycles are reached, motion automatically moves to center position and stops there (amplitude = 0).

### Start/Stop Control

Motion can be started and stopped at any time:

```cpp
// Start motion (can be called at any time)
motion.Start();

// Stop motion (can be called at any time)
motion.Stop();

// Check if running
if (motion.IsRunning()) {
    // Motion is active
}

// Resume after stop (continues from current position)
motion.Start();
```

### Real-Time Setting Changes

All settings can be changed while motion is running:

```cpp
// Change frequency in real-time
motion.SetFrequency(1.0f);  // Increase to 1.0 Hz

// Change amplitude in real-time
motion.SetSinuousAmplitudeDegrees(45.0f);  // Reduce to 45°

// Change dwell times in real-time
motion.SetDwellTimes(1000, 1000, 0);

// Change target cycles in real-time
motion.SetTargetCycles(2000);
```

### Complete Example Flow

1. **Find Global Bounds**: Automatically detects mechanical stops in both directions
2. **Set Home**: Sets middle position as home (or uses current position if unbounded)
3. **Set Local Bounds**: Define oscillation range (automatically clipped to global bounds)
4. **Configure**: Set up sinuous motion parameters, dwell times, and cycle count
5. **Start Motion**: Begin pure sinusoidal back-and-forth motion
6. **Update Loop**: Continuously update motion controller
7. **Monitor**: Track cycle count and adjust settings in real-time as needed
8. **Stop**: Motion stops automatically when cycle count reached, or can be stopped manually

### Fatigue Testing Best Practices

For cable/strain relief fatigue testing:

- **Pure sinusoidal motion** between two bounds is ideal for worst-case fatigue testing
- **Dwell at extremes** (1-5 seconds) simulates holding tool in awkward positions
- **No center dwell** typically needed for pure fatigue testing
- **Constant frequency** maximizes cycles per hour for faster test completion
- **Angle selection**: Use realistic but aggressive angles (e.g., ±60-90°)

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
- **Search Speed**: Adjust based on your application (typically 0.004-0.02 rev/s, ~0.24-1.2 RPM for typical motors)
- **Global vs Local Bounds**: 
  - Global bounds are hardware limits (found during initialization)
  - Local bounds define oscillation range (clipped to global bounds automatically)
  - If local bounds exceed global bounds, they are automatically clipped
- **Dwell Times**: 
  - Set to 0 to disable dwells for continuous motion
  - Typical values: 1-5 seconds at extremes for fatigue testing
  - Center dwell is optional and typically not needed for pure fatigue testing
  - Can be changed in real-time while motion is running
- **Cycle Count**:
  - Set target cycles (0 = infinite)
  - **One cycle = center → min → max → center** (or center → max → min → center)
  - Cycles are counted at center crossing point (0 crossing)
  - Motion automatically stops at center position when target cycles reached
  - Cycle count can be changed in real-time
  - Use `ResetCycles()` to restart counting
- **Start/Stop Control**:
  - Motion can be started and stopped at any time
  - Resuming after stop continues from current position
  - All settings can be changed while motion is running

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

