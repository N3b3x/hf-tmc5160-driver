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
#include "tmc51x0.hpp"  // Main driver header - includes unit types automatically
// Or for standalone conversions:
#include "features/tmc51x0_unit_types.hpp"
```

### Type-Safe Unit System

The driver provides type-safe unit wrappers (`Position`, `Velocity`, `Acceleration`) with dedicated unit enums that prevent mixing incompatible units:

```cpp
// Create type-safe values with factory methods
auto pos = tmc51x0::Position::Mm(10.0f);           // 10mm position
auto vel = tmc51x0::Velocity::RPM(60.0f);          // 60 RPM velocity
auto accel = tmc51x0::Acceleration::RevPerSec2(2.0f); // 2 rev/s² acceleration

// Convert using ConversionParams
auto params = tmc51x0::ConversionParams::LeadScrew(200, 2.0f);  // 200 steps/rev, 2mm pitch
float steps = pos.toSteps(params);
float steps_per_sec = vel.toStepsPerSec(params);
```

### Lead Screw Example

```cpp
// Setup conversion params for your mechanical system
auto params = tmc51x0::ConversionParams::LeadScrew(200, 2.0f);  // 200 steps/rev, 2mm pitch

// Convert 10mm to steps using type-safe Position
auto pos = tmc51x0::Position::Mm(10.0f);
float steps = pos.toSteps(params);

// Or use driver's built-in unit conversion (requires mechanical system in DriverConfig)
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

// Or convert manually using ConversionParams
auto params = tmc51x0::ConversionParams::DirectDrive(STEPS_PER_REV);
float steps_per_sec = tmc51x0::RpmToStepsPerSec(30.0f, params);
driver.rampControl.SetMaxSpeed(steps_per_sec, tmc51x0::Unit::Steps);
```

### Belt Drive Example

```cpp
// Setup conversion params for belt drive
auto params = tmc51x0::ConversionParams::BeltDrive(200, 2.0f, 20);  // 200 steps/rev, 2mm pitch, 20 teeth

// Convert position using type-safe Position
auto pos = tmc51x0::Position::Revolutions(5.0f);  // 5 revolutions = 100 teeth
float steps = pos.toSteps(params);
driver.rampControl.SetTargetPosition(steps);
```

## Type-Safe Unit Conversion System

The recommended approach uses type-safe structs with dedicated unit enums:

### Position Conversions

| Method | Description | Example |
|--------|-------------|---------|
| `Position::Mm(v)` | Create mm position | `Position::Mm(10.0f)` |
| `Position::Deg(v)` | Create degrees position | `Position::Deg(90.0f)` |
| `Position::Revolutions(v)` | Create revolutions position | `Position::Revolutions(2.5f)` |
| `pos.toSteps(params)` | Convert to steps | `pos.toSteps(params)` |
| `pos.convertTo(unit, params)` | Convert to any unit | `pos.convertTo(PositionUnit::Mm, params)` |

### Velocity Conversions

| Method | Description | Example |
|--------|-------------|---------|
| `Velocity::RPM(v)` | Create RPM velocity | `Velocity::RPM(60.0f)` |
| `Velocity::RevPerSec(v)` | Create rev/s velocity | `Velocity::RevPerSec(1.0f)` |
| `Velocity::MmPerSec(v)` | Create mm/s velocity | `Velocity::MmPerSec(10.0f)` |
| `vel.toStepsPerSec(params)` | Convert to steps/s | `vel.toStepsPerSec(params)` |
| `vel.convertTo(unit, params)` | Convert to any unit | `vel.convertTo(VelocityUnit::RPM, params)` |

**Note**: The driver API still supports `Unit::RevPerSec` (revolutions per second) as a velocity unit. For type-safe code, use `Velocity` struct:
- `Velocity::RevPerSec(0.5f)` creates 0.5 rev/s (30 RPM)
- `Velocity::RPM(30.0f)` creates 30 RPM (0.5 rev/s)
- Convert between units: `vel.convertTo(VelocityUnit::DegPerSec, params)`

### Acceleration Conversions

| Method | Description | Example |
|--------|-------------|---------|
| `Acceleration::RevPerSec2(v)` | Create rev/s² accel | `Acceleration::RevPerSec2(2.0f)` |
| `Acceleration::MmPerSec2(v)` | Create mm/s² accel | `Acceleration::MmPerSec2(100.0f)` |
| `accel.toStepsPerSec2(params)` | Convert to steps/s² | `accel.toStepsPerSec2(params)` |

### Standalone Conversion Functions

For convenience, standalone functions are also provided:

| Function | Description | Example |
|----------|-------------|---------|
| `MmToSteps(mm, params)` | Convert mm to steps | `MmToSteps(10.0f, params)` |
| `StepsToMm(steps, params)` | Convert steps to mm | `StepsToMm(1000, params)` |
| `DegreesToSteps(deg, params)` | Convert degrees to steps | `DegreesToSteps(90.0f, params)` |
| `RpmToStepsPerSec(rpm, params)` | Convert RPM to steps/s | `RpmToStepsPerSec(60.0f, params)` |
| `StepsPerSecToRpm(sps, params)` | Convert steps/s to RPM | `StepsPerSecToRpm(200.0f, params)` |

## Complete Example

```cpp
#include "tmc51x0.hpp"
#include "features/tmc51x0_unit_types.hpp"

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
        return;
    }
    
    // Create ConversionParams for standalone conversions
    auto params = tmc51x0::ConversionParams::LeadScrew(STEPS_PER_REV, LEAD_SCREW_PITCH_MM);
    
    // Use type-safe Velocity (recommended)
    auto vel = tmc51x0::Velocity::RPM(100.0f);
    float steps_per_sec = vel.toStepsPerSec(params);  // Convert if needed
    
    // Or use driver API with legacy Unit enum
    auto rpm_result = driver.rampControl.SetMaxSpeed(100.0f, tmc51x0::Unit::RPM);
    if (!rpm_result) {
        printf("Error setting max speed: %s\n", rpm_result.ErrorMessage());
        return;
    }
    
    // Use type-safe Acceleration (recommended)
    auto accel = tmc51x0::Acceleration::RevPerSec2(2.0f);  // 2 rev/s²
    float accel_steps = accel.toStepsPerSec2(params);
    
    // Or use driver API with legacy Unit enum
    auto accel_result = driver.rampControl.SetAcceleration(50.0f, tmc51x0::Unit::Mm);  // 50 mm/s²
    if (!accel_result) {
        printf("Error setting acceleration: %s\n", accel_result.ErrorMessage());
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

