---
layout: default
title: "📐 Unit Conversions"
description: "Converting between physical units and driver steps"
nav_order: 9
parent: "📚 Documentation"
permalink: /docs/special_features_unit_conversions/
---

# Unit Conversions

The TMC5160 driver works internally with steps, but most applications need to work with physical units like millimeters, degrees, or RPM. This guide shows how to use the unit conversion functions to work with intuitive units.

## Overview

The driver provides free functions in the `tmc5160` namespace for converting between:
- **Physical units**: millimeters, degrees, RPM, mm/s, mm/s²
- **Driver units**: steps, steps/s, steps/s²

All conversion functions require knowledge of your motor's steps per revolution and mechanical system parameters (e.g., lead screw pitch).

## When to Use Unit Conversions

- ✅ Setting target positions in millimeters for lead screw systems
- ✅ Setting speeds in RPM instead of steps per second
- ✅ Working with belt drives using pulley teeth
- ✅ Converting encoder readings to physical positions
- ❌ Low-level register access (use raw steps)
- ❌ When maximum performance is critical (direct step calculations may be faster)

## Basic Usage

### Including the Header

```cpp
#include "tmc5160_units.hpp"
```

### Lead Screw Example

```cpp
// Motor: 200 steps/rev, Lead screw: 2mm pitch
constexpr uint16_t STEPS_PER_REV = 200;
constexpr float LEAD_SCREW_PITCH_MM = 2.0f;

// Move 10mm
int32_t steps = tmc5160::MmToSteps(10.0f, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
driver.rampControl.SetTargetPosition(steps);

// Or use the convenience function
driver.rampControl.SetTargetPositionMm(10.0f, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);

// Get current position in mm
int32_t current_steps = driver.rampControl.GetCurrentPosition();
float current_mm = tmc5160::StepsToMm(current_steps, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
```

### Speed in RPM

```cpp
// Set maximum speed to 100 RPM
driver.rampControl.SetMaxSpeedRpm(100.0f, STEPS_PER_REV);

// Or convert manually
float steps_per_sec = tmc5160::RpmToStepsPerSec(100.0f, STEPS_PER_REV);
driver.rampControl.SetMaxSpeed(steps_per_sec);
```

### Belt Drive Example

```cpp
// Motor: 200 steps/rev, Pulley: 20 teeth
constexpr uint16_t PULLEY_TEETH = 20;

// Move 100 belt teeth
int32_t steps = tmc5160::BeltTeethToSteps(100, STEPS_PER_REV, PULLEY_TEETH);
driver.rampControl.SetTargetPosition(steps);
```

## Available Conversion Functions

### Position Conversions

| Function | Description | Example |
|----------|-------------|---------|
| `MmToSteps()` | Convert mm to steps | `MmToSteps(10.0f, 200, 2.0f)` |
| `StepsToMm()` | Convert steps to mm | `StepsToMm(1000, 200, 2.0f)` |
| `DegreesToSteps()` | Convert degrees to steps | `DegreesToSteps(90.0f, 200)` |
| `StepsToDegrees()` | Convert steps to degrees | `StepsToDegrees(50, 200)` |
| `BeltTeethToSteps()` | Convert belt teeth to steps | `BeltTeethToSteps(100, 200, 20)` |
| `StepsToBeltTeeth()` | Convert steps to belt teeth | `StepsToBeltTeeth(1000, 200, 20)` |

### Speed Conversions

| Function | Description | Example |
|----------|-------------|---------|
| `RpmToStepsPerSec()` | Convert RPM to steps/s | `RpmToStepsPerSec(100.0f, 200)` |
| `StepsPerSecToRpm()` | Convert steps/s to RPM | `StepsPerSecToRpm(333.3f, 200)` |
| `MmPerSecToStepsPerSec()` | Convert mm/s to steps/s | `MmPerSecToStepsPerSec(10.0f, 200, 2.0f)` |
| `StepsPerSecToMmPerSec()` | Convert steps/s to mm/s | `StepsPerSecToMmPerSec(333.3f, 200, 2.0f)` |

### Acceleration Conversions

| Function | Description | Example |
|----------|-------------|---------|
| `AccelerationMmToSteps()` | Convert mm/s² to steps/s² | `AccelerationMmToSteps(100.0f, 200, 2.0f)` |
| `AccelerationStepsToMm()` | Convert steps/s² to mm/s² | `AccelerationStepsToMm(5000.0f, 200, 2.0f)` |

## Complete Example

```cpp
#include "tmc5160.hpp"
#include "tmc5160_units.hpp"

// Motor and mechanical system parameters
constexpr uint16_t STEPS_PER_REV = 200;      // 1.8° stepper
constexpr float LEAD_SCREW_PITCH_MM = 2.0f;  // 2mm pitch lead screw

void setupMotor() {
    // Initialize driver
    tmc5160::DriverConfig cfg{};
    driver.Initialize(cfg);
    
    // Set speeds in RPM
    driver.rampControl.SetMaxSpeedRpm(100.0f, STEPS_PER_REV);
    
    // Set acceleration in mm/s²
    float accel_mm_per_sec2 = 50.0f;
    float accel_steps_per_sec2 = tmc5160::AccelerationMmToSteps(
        accel_mm_per_sec2, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
    driver.rampControl.SetAcceleration(accel_steps_per_sec2);
}

void moveToPosition(float target_mm) {
    // Move to position in millimeters
    driver.rampControl.SetTargetPositionMm(
        target_mm, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
    
    // Wait for completion
    while (!driver.rampControl.IsTargetReached()) {
        // Monitor position in mm
        int32_t steps = driver.rampControl.GetCurrentPosition();
        float mm = tmc5160::StepsToMm(steps, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
        printf("Current position: %.2f mm\n", mm);
    }
}
```

## Performance Considerations

- **Compile-time evaluation**: All conversion functions are `constexpr` and can be evaluated at compile time for constant inputs
- **Runtime overhead**: Minimal - simple arithmetic operations
- **Precision**: Uses `float` for physical units, `int32_t` for steps

## Troubleshooting

### Incorrect Movement Distance

**Problem**: Motor moves wrong distance

**Solution**: 
- Verify `steps_per_rev` matches your motor (typically 200 for 1.8° motors)
- Check mechanical system parameters (lead screw pitch, pulley teeth)
- Ensure consistent units (mm vs inches)

### Speed Too Fast/Slow

**Problem**: Motor speed doesn't match expected RPM

**Solution**:
- Verify `steps_per_rev` is correct
- Check that microstep resolution matches your chopper configuration
- Remember: internal calculations use microsteps (256 per step)

---

**Navigation**
⬅️ [Previous: Examples](examples.md) | [Next: Motor Setup ➡️](special_features_motor_setup.md) | [Docs Hub 📚](index.md)

