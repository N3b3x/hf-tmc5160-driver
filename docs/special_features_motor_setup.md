---
layout: default
title: "⚙️ Motor Setup from Specifications"
description: "High-level motor configuration from physical parameters"
nav_order: 10
parent: "📚 Documentation"
permalink: /docs/special_features_motor_setup/
---

# Motor Setup from Specifications

Instead of manually calculating `irun`, `ihold`, and `global_scaler` values, you can use `SetupMotorFromSpec()` to automatically configure the driver from your motor's physical specifications.

## Overview

The `SetupMotorFromSpec()` function takes high-level motor parameters (rated current, steps per revolution, etc.) and automatically calculates the appropriate driver settings. This makes it much easier to get started, especially for users unfamiliar with the low-level driver parameters.

## When to Use Motor Setup from Specs

- ✅ Quick prototyping and initial setup
- ✅ When you know motor specifications but not driver parameters
- ✅ Educational projects and learning
- ✅ Applications where exact current control isn't critical
- ❌ Production systems requiring precise current control
- ❌ When you need fine-tuned performance optimization

## Basic Usage

### Simple Setup

```cpp
#include "tmc51x0.hpp"

// Define motor specifications
tmc51x0::MotorSpec motor_spec{};
motor_spec.steps_per_rev = 200;        // 1.8° stepper motor
motor_spec.rated_current_ma = 1500;     // 1.5A rated current

// Setup motor automatically
auto setup_result = driver.motorControl.SetupMotorFromSpec(motor_spec);
if (!setup_result) {
    printf("Error setting up motor: %s\n", setup_result.ErrorMessage());
    return;
}

// Motor is now configured and ready to use!
auto enable_result = driver.motorControl.Enable();
if (!enable_result) {
    printf("Error enabling motor: %s\n", enable_result.ErrorMessage());
    return;
}
```

### With Mechanical System

```cpp
// Define mechanical system
tmc51x0::MechanicalSystem mech_system{};
mech_system.system_type = tmc51x0::MechanicalSystemType::LeadScrew;
mech_system.lead_screw_pitch_mm = 2.0f;  // 2mm pitch

// Setup with mechanical system info
driver.motorControl.SetupMotorFromSpec(motor_spec, &mech_system);
```

## Motor Specification Structure

### Required Parameters

```cpp
struct MotorSpec {
    uint16_t steps_per_rev;      // Steps per revolution (typically 200 for 1.8°)
    uint16_t rated_current_ma;   // Rated current in milliamps (RMS)
    uint32_t sense_resistor_mohm; // Sense resistor value in milliohms (e.g., 50 for 0.05Ω)
    uint32_t supply_voltage_mv;   // Motor supply voltage in millivolts (e.g., 24000 for 24V)
};
```

### Optional Parameters (for Advanced Configuration)

```cpp
struct MotorSpec {
    // ... required parameters ...
    
    uint32_t winding_resistance_mohm;  // Winding resistance in milliohms (required for StealthChop)
    float winding_inductance_mh;       // Winding inductance in millihenries (0 = not specified)
    uint16_t run_current_ma;          // Desired run current (0 = use rated_current_ma)
    uint16_t hold_current_ma;          // Desired hold current (0 = auto-calculate as 30% of run)
    float scaler_adjustment_percent;   // Fine-tuning for GLOBAL_SCALER calculation (-50.0 to +50.0)
    float irun_adjustment_percent;     // Fine-tuning for IRUN calculation (-50.0 to +50.0)
    float ihold_adjustment_percent;    // Fine-tuning for IHOLD calculation (-50.0 to +50.0)
};
```

**Note**: Both `Initialize()` with `DriverConfig` and `SetupMotorFromSpec()` use the same accurate calculation methods based on datasheet formulas. `Initialize()` is recommended for complete driver setup, while `SetupMotorFromSpec()` is useful for updating motor settings after initialization.

## How It Works

The `SetupMotorFromSpec()` function uses the same accurate calculation methods as `Initialize()`:

1. **Calculates Motor Current Settings**: Uses datasheet formulas to calculate IRUN, IHOLD, and GLOBAL_SCALER
   - Based on sense resistor value, supply voltage, and desired current
   - Uses proper datasheet formula: `I_RMS = (GLOBAL_SCALER/256) * ((CS+1)/32) * (VFS/RSENSE) * (1/√2)`
   - Optimizes IRUN to be in the 16-31 range for best performance
   - Defaults to 80% of rated current for run current, 30% for hold current (if not specified)

2. **Configures Chopper**: Automatically adjusts chopper settings based on motor inductance (if provided)
   - Reduces power consumption when motor is stationary

4. **Configures Chopper** (if inductance specified):
   - Adjusts blank time for high-inductance motors

## Complete Example

```cpp
#include "tmc51x0.hpp"

void setupNema17Motor() {
    // NEMA 17 motor specifications
    tmc51x0::MotorSpec nema17{};
    nema17.steps_per_rev = 200;           // 1.8° per step
    nema17.rated_current_ma = 1500;       // 1.5A rated
    nema17.winding_resistance_mohm = 3200; // 3.2Ω per phase
    nema17.winding_inductance_mh = 2.8f;   // 2.8 mH per phase
    
    // Lead screw mechanical system
    tmc51x0::MechanicalSystem lead_screw{};
    lead_screw.system_type = tmc51x0::MechanicalSystemType::LeadScrew;
    lead_screw.lead_screw_pitch_mm = 2.0f;
    
    // Initialize driver
    tmc51x0::DriverConfig cfg{};
    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Initialization error: %s\n", init_result.ErrorMessage());
        return;
    }
    
    // Setup motor from specifications
    auto setup_result2 = driver.motorControl.SetupMotorFromSpec(nema17, &lead_screw);
    if (!setup_result2) {
        printf("Failed to setup motor: %s\n", setup_result2.ErrorMessage());
        return;
    }
    
    // Configure motion parameters
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetMaxSpeed(100.0f, tmc51x0::Unit::Rpm, nema17.steps_per_rev);
    driver.rampControl.SetAcceleration(500.0f, tmc51x0::Unit::RpmPerSecond, nema17.steps_per_rev);
    
    // Enable motor
    driver.motorControl.Enable();
}
```

## Manual Current Configuration

If you need precise control, you can still set current manually:

```cpp
// Set current directly (irun=20, ihold=10)
driver.motorControl.SetCurrent(20, 10);

// Or use MotorCurrentConfig for physical units
tmc51x0::MotorCurrentConfig current_cfg{};
current_cfg.run_current_ma = 1200;      // 1.2A run current
current_cfg.hold_current_ma = 400;      // 400mA hold current
current_cfg.hold_current_delay_ms = 10;  // 10ms delay before reducing current

// Calculate and set (you would implement this helper)
// driver.motorControl.SetCurrentFromMa(current_cfg);
```

## Mechanical System Types

### Direct Drive

```cpp
tmc51x0::MechanicalSystem direct{};
direct.system_type = tmc51x0::MechanicalSystemType::DirectDrive;
// No additional parameters needed
```

### Lead Screw

```cpp
tmc51x0::MechanicalSystem lead_screw{};
lead_screw.system_type = tmc51x0::MechanicalSystemType::LeadScrew;
lead_screw.lead_screw_pitch_mm = 2.0f;  // 2mm pitch
```

### Belt Drive

```cpp
tmc51x0::MechanicalSystem belt{};
belt.system_type = tmc51x0::MechanicalSystemType::BeltDrive;
belt.belt_pulley_teeth = 20;            // 20-tooth motor pulley
belt.belt_pitch_mm = 2.0f;              // 2mm belt pitch (GT2)
```

### Gearbox

```cpp
tmc51x0::MechanicalSystem gearbox{};
gearbox.system_type = tmc51x0::MechanicalSystemType::Gearbox;
gearbox.gear_ratio = 10.0f;  // 10:1 reduction (output/input)
```

## Troubleshooting

### Motor Doesn't Move

**Problem**: Motor configured but doesn't move

**Solution**:
- Verify `rated_current_ma` is correct (check motor datasheet)
- Ensure `irun` is at least 16 (check calculated value)
- Verify motor is enabled: `driver.motorControl.Enable()`
- Check chopper is enabled (`toff > 0`)

### Motor Runs Too Hot

**Problem**: Motor gets hot during operation

**Solution**:
- Reduce `rated_current_ma` in `MotorSpec` (setup will recalculate)
- Or manually reduce `irun` value
- Check that `ihold` is reasonable (30% of run current)

### Insufficient Torque

**Problem**: Motor stalls under load

**Solution**:
- Increase `rated_current_ma` in `MotorSpec`
- Verify power supply can deliver required current
- Check mechanical system isn't binding

## Advanced: Custom Current Calculation

For production systems, you may want to implement your own current calculation:

```cpp
uint8_t calculateIrun(uint16_t target_current_ma, uint16_t global_scaler) {
    // Your custom calculation based on sense resistor, etc.
    // This is a simplified example
    float sense_resistor_current = 1.5f; // Amps at irun=31, scaler=32
    float target_amps = target_current_ma / 1000.0f;
    uint8_t irun = static_cast<uint8_t>(
        (target_amps * 32.0f) / (sense_resistor_current * global_scaler / 32.0f));
    return std::min(31U, std::max(16U, static_cast<uint32_t>(irun)));
}
```

---

**Navigation**
⬅️ [Previous: Unit Conversions](special_features_unit_conversions.md) | [Next: Sensorless Homing ➡️](special_features_sensorless_homing.md) | [Docs Hub 📚](index.md)

