---
layout: default
title: "📐 Unit Conversions"
description: "Converting between physical units and driver steps"
nav_order: 9
parent: "📚 Documentation"
permalink: /docs/special_features_unit_conversions/
---

# Unit Conversions

The TMC51x0 driver (TMC5130 & TMC5160) works internally with steps, but most applications need to work with physical units like millimeters, degrees, RPM, or revolutions per second. This guide shows how to use the unit conversion functions and unit-aware API to work with intuitive units.

**Important**: All unit-aware functions require explicit unit specification. Always specify the unit parameter to ensure clarity and prevent accidental use of step-based values.

## Overview

The driver provides unit-aware functions and free conversion functions for working with:
- **Physical units**: millimeters, degrees, RPM, revolutions per second (RevPerSec), mm/s, mm/s²
- **Driver units**: steps, steps/s, steps/s²

**Note**: All unit-aware functions require explicit unit specification. Recommended units: `Unit::Deg` for position, `Unit::RPM` for velocity, and `Unit::RevPerSec` for acceleration.

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
#include "tmc51x0/features/tmc51x0_units.hpp"
```

### Lead Screw Example

```cpp
// Motor: 200 steps/rev, Lead screw: 2mm pitch
constexpr uint16_t STEPS_PER_REV = 200;
constexpr float LEAD_SCREW_PITCH_MM = 2.0f;

// Move 10mm
int32_t steps = tmc51x0::MmToSteps(10.0f, STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
driver.rampControl.SetTargetPosition(steps);

// Or use SetTargetPosition with Unit parameter (requires mechanical system in DriverConfig)
driver.rampControl.SetTargetPosition(10.0f, tmc51x0::Unit::Mm);

// Get current position in mm
auto pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Mm);
if (pos_result) {
    float current_mm = pos_result.Value();
    // Use current_mm
} else {
    printf("Error reading position: %s\n", pos_result.ErrorMessage());
}
```

### Speed in Revolutions Per Second (Recommended Default)

```cpp
// Set maximum speed to 0.5 rev/s (30 RPM) - Unit::RevPerSec is now the default
driver.rampControl.SetMaxSpeed(0.5f);  // No unit parameter needed!

// Or explicitly specify the unit
driver.rampControl.SetMaxSpeed(0.5f, tmc51x0::Unit::RevPerSec);

// Or use RPM (converts to rev/s internally)
driver.rampControl.SetMaxSpeed(30.0f, tmc51x0::Unit::Rpm);  // 30 RPM = 0.5 rev/s

// Or use degrees per second
driver.rampControl.SetMaxSpeed(180.0f, tmc51x0::Unit::Deg);  // 180 deg/s = 0.5 rev/s

// Or convert manually (if needed)
float steps_per_sec = tmc51x0::RpmToStepsPerSec(30.0f, STEPS_PER_REV);
driver.rampControl.SetMaxSpeed(steps_per_sec, tmc51x0::Unit::Steps);
```

### Belt Drive Example

```cpp
// Motor: 200 steps/rev, Pulley: 20 teeth
constexpr uint16_t PULLEY_TEETH = 20;

// Move 100 belt teeth
int32_t steps = tmc51x0::BeltTeethToSteps(100, STEPS_PER_REV, PULLEY_TEETH);
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

**Note**: The driver now supports `Unit::RevPerSec` (revolutions per second) as the default unit for velocity functions. This makes it much easier to work with intuitive units without manual conversions. For example:
- `SetMaxSpeed(0.5f)` sets speed to 0.5 rev/s (30 RPM) - no unit parameter needed!
- `SetMaxSpeed(30.0f, Unit::Rpm)` also works and converts automatically
- `SetMaxSpeed(180.0f, Unit::Deg)` sets speed to 180 deg/s (0.5 rev/s)

### Acceleration Conversions

| Function | Description | Example |
|----------|-------------|---------|
| `AccelerationMmToSteps()` | Convert mm/s² to steps/s² | `AccelerationMmToSteps(100.0f, 200, 2.0f)` |
| `AccelerationStepsToMm()` | Convert steps/s² to mm/s² | `AccelerationStepsToMm(5000.0f, 200, 2.0f)` |

## Complete Example

```cpp
#include "tmc51x0.hpp"
#include "tmc51x0/features/tmc51x0_units.hpp"

// Motor and mechanical system parameters
constexpr uint16_t STEPS_PER_REV = 200;      // 1.8° stepper
constexpr float LEAD_SCREW_PITCH_MM = 2.0f;  // 2mm pitch lead screw

void setupMotor() {
    // Initialize driver with mechanical system configuration
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.steps_per_rev = STEPS_PER_REV;
    cfg.mechanical.system_type = tmc51x0::MechanicalSystemType::LeadScrew;
    cfg.mechanical.lead_screw_pitch_mm = LEAD_SCREW_PITCH_MM;
    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Initialization error: %s\n", init_result.ErrorMessage());
        return; // or handle error appropriately
    }
    
    // Set speeds in revolutions per second (default unit - no parameter needed!)
    auto speed_result = driver.rampControl.SetMaxSpeed(1.67f);  // 1.67 rev/s = 100 RPM - Unit::RevPerSec is default
    if (!speed_result) {
        printf("Error setting max speed: %s\n", speed_result.ErrorMessage());
        return;
    }
    
    // Or use RPM (converts automatically)
    auto rpm_result = driver.rampControl.SetMaxSpeed(100.0f, tmc51x0::Unit::Rpm);
    if (!rpm_result) {
        printf("Error setting max speed: %s\n", rpm_result.ErrorMessage());
        return;
    }
    
    // Set acceleration in rev/s² (default unit)
    auto accel_result = driver.rampControl.SetAcceleration(0.83f);  // 0.83 rev/s² - Unit::RevPerSec is default
    if (!accel_result) {
        printf("Error setting acceleration: %s\n", accel_result.ErrorMessage());
        return;
    }
    
    // Or use mm/s² (uses mechanical system from DriverConfig)
    auto accel_mm_result = driver.rampControl.SetAcceleration(50.0f, tmc51x0::Unit::Mm);  // 50 mm/s²
    if (!accel_mm_result) {
        printf("Error setting acceleration: %s\n", accel_mm_result.ErrorMessage());
        return;
    }
}

void moveToPosition(float target_mm) {
    // Move to position in millimeters (uses mechanical system from DriverConfig)
    auto pos_result = driver.rampControl.SetTargetPosition(target_mm, tmc51x0::Unit::Mm);
    if (!pos_result) {
        printf("Error setting target position: %s\n", pos_result.ErrorMessage());
        return; // or handle error appropriately
    }
    
    // Wait for completion
    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached && reached.Value()) {
            break; // Target reached
        }
        if (!reached) {
            printf("Error checking target: %s\n", reached.ErrorMessage());
            break;
        }
        // Monitor position in mm (uses mechanical system from DriverConfig)
        auto pos_result2 = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Mm);
        if (pos_result2) {
            printf("Current position: %.2f mm\n", pos_result2.Value());
        } else {
            printf("Error reading position: %s\n", pos_result2.ErrorMessage());
            // Continue monitoring despite read error
        }
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
- Check the configured microstep resolution (`chopper.mres`)
- If you change microstep resolution (MRES) at runtime, do it at standstill:
  - The driver will preserve physical meaning by default (position + ramp profile are rescaled)
  - If you intentionally want a \"raw\" change (no rescaling), use `SetMicrostepResolution()` with `MicrostepChangeOptions{ .preserve_physical_units = false }`

### Quick Verification: MRES Change Preserves Physical Units

At standstill:
- Record position in full steps: `GetCurrentPosition(Unit::Steps)`
- Record configured max speed in RPM: `GetCurrentSpeed(Unit::RPM)` (or track your configured VMAX)
- Change MRES (e.g. 256 → 128) using `SetMicrostepResolution()`
- Verify:
  - `GetCurrentPosition(Unit::Steps)` is unchanged
  - A move of N full steps still moves the same physical distance

### Note on Quantization and Rounding

All motion and position settings are ultimately written to integer-valued driver registers.\nThis means values expressed in real units (RPM/deg/mm) are **quantized** to the nearest representable register value.\nThe driver uses a consistent policy of **round-to-nearest (ties away from zero)** when converting float inputs to register values.\n 

---

**Navigation**
⬅️ [Previous: Examples](examples.md) | [Next: Motor Setup ➡️](special_features_motor_setup.md) | [Docs Hub 📚](index.md)

