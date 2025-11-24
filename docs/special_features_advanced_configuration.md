---
layout: default
title: "🔧 Advanced Configuration"
description: "Advanced features: CoolStep, dcStep, freewheeling, and more"
nav_order: 12
parent: "📚 Documentation"
permalink: /docs/special_features_advanced_configuration/
---

# Advanced Configuration

This guide covers advanced TMC5160 features including CoolStep current reduction, dcStep automatic commutation, freewheeling modes, reference switches, and microstep lookup tables.

## Overview

The TMC5160 provides several advanced features for optimizing motor performance:

- **CoolStep**: Automatically reduces current when load is low
- **dcStep**: Automatic commutation for DC motor-like operation
- **Freewheeling**: Low-power modes when motor is stationary
- **Reference Switches**: Endstop support for homing and limits
- **Microstep Lookup Tables**: Custom microstep interpolation

## CoolStep Current Reduction

CoolStep automatically reduces motor current when the load is low, reducing power consumption and heat generation.

### When to Use CoolStep

- ✅ Applications with varying load
- ✅ Battery-powered systems
- ✅ When motor runs hot
- ✅ Long-running applications
- ❌ Constant high-load applications
- ❌ When precise current control is needed

### Configuration

```cpp
#include "tmc5160.hpp"

void configureCoolStep() {
    tmc5160::CoolStepConfig coolstep{};
    coolstep.semin = 2;   // Minimum SG value to enable CoolStep (0-15)
    coolstep.semax = 5;   // Hysteresis for CoolStep (0-15)
    coolstep.seup = 1;    // Current increment step (0-3)
    coolstep.sedn = 2;    // Current decrement step (0-3)
    coolstep.seimin = false;  // Minimum current (false = use ihold)
    coolstep.sfilt = true;    // Enable StallGuard filter
    
    driver.motorControl.ConfigureCoolStep(coolstep);
    
    // Set velocity threshold for CoolStep activation
    // CoolStep activates between TCOOLTHRS and THIGH
    // IMPORTANT: CoolStep requires SpreadCycle mode (en_pwm_mode=0)
    driver.motorControl.SetModeChangeSpeeds(
        1000.0f,  // TPWMTHRS: stealthChop threshold
        500.0f,   // TCOOLTHRS: CoolStep lower threshold
        5000.0f   // THIGH: CoolStep upper threshold
    );
}
```

### How CoolStep Works

1. **StallGuard2 Monitoring**: Continuously monitors motor load
2. **Current Reduction**: When load is low (high SG value), reduces current
3. **Current Increase**: When load increases (low SG value), increases current
4. **Hysteresis**: `semax` prevents rapid current changes

**Note**: CoolStep requires StallGuard2, which ONLY works in SpreadCycle mode. CoolStep is automatically disabled if StealthChop is enabled (`en_pwm_mode=1`).

## dcStep Automatic Commutation

dcStep enables automatic commutation for smoother operation at low speeds, similar to DC motor control.

### When to Use dcStep

- ✅ Low-speed applications requiring smooth motion
- ✅ Applications where step loss can be tolerated
- ✅ When smoother motion is more important than precision
- ❌ High-precision positioning applications
- ❌ When step loss is unacceptable

### Configuration

```cpp
void configureDcStep() {
    tmc5160::DcStepConfig dcstep{};
    // Set velocity threshold (0.0f = disabled)
    // dcStep activates below this threshold
    dcstep.vdc_min = 1000.0f;  // steps/s threshold
    
    driver.motorControl.ConfigureDcStep(dcstep);
    
    // Note: dcStep requires SD_MODE=1 (external step/dir mode)
    // This is typically set in GCONF register
}
```

## Freewheeling Mode

Freewheeling controls motor behavior when `ihold=0` (no hold current).

### Freewheeling Options

```cpp
void configureFreewheeling() {
    // Normal operation (coast when ihold=0)
    driver.motorControl.SetFreewheelingMode(tmc5160::PWMFreewheel::NORMAL);
    
    // Freewheeling enabled (low resistance)
    driver.motorControl.SetFreewheelingMode(tmc5160::PWMFreewheel::ENABLED);
    
    // Coil shorted using low-side drivers
    driver.motorControl.SetFreewheelingMode(tmc5160::PWMFreewheel::SHORT_LS);
    
    // Coil shorted using high-side drivers
    driver.motorControl.SetFreewheelingMode(tmc5160::PWMFreewheel::SHORT_HS);
}
```

### Use Cases

- **NORMAL**: Standard operation, motor coasts
- **ENABLED**: Low power consumption, easy to turn
- **SHORT_LS/HS**: Braking effect, motor resists rotation

## Reference Switches (Endstops)

Reference switches provide hardware endstops for homing and limit detection.

### Configuration

```cpp
void configureEndstops() {
    tmc5160::ReferenceSwitchConfig ref_switch{};
    
    // Enable left endstop
    ref_switch.stop_left_enable = true;
    ref_switch.pol_stop_left = false;  // Active high
    
    // Enable right endstop
    ref_switch.stop_right_enable = true;
    ref_switch.pol_stop_right = false;  // Active high
    
    // Latch position on switch activation
    ref_switch.latch_left_active = true;
    ref_switch.latch_right_active = true;
    
    // Enable soft stop (uses deceleration ramp)
    ref_switch.en_softstop = true;
    
    driver.rampControl.ConfigureReferenceSwitch(ref_switch);
}
```

### Reading Latched Position

```cpp
void homeToEndstop() {
    // Move toward endstop
    driver.rampControl.SetRampMode(tmc5160::RampMode::VELOCITY_NEG);
    driver.rampControl.SetMaxSpeed(500.0f);
    driver.motorControl.Enable();
    
    // Wait for endstop (check RAMP_STAT register)
    // Or use GetLatchedPosition() after switch triggers
    
    int32_t latched_pos = driver.rampControl.GetLatchedPosition();
    driver.rampControl.SetCurrentPosition(0);  // Set as home
}
```

## Microstep Lookup Tables

Custom microstep lookup tables allow fine-tuning of microstep interpolation for smoother motion.

### Default Lookup Table

The TMC5160 uses a default sinusoidal lookup table. For most applications, this is sufficient.

### Custom Lookup Table

```cpp
void configureCustomMicrosteps() {
    // Set lookup table entries (8 entries, 32 bits each)
    // Each entry defines current for a microstep segment
    
    // Example: Custom table for smoother motion
    uint32_t lut_values[8] = {
        0xAAAAB554,  // Entry 0
        0xAAAAB554,  // Entry 1
        0xAAAAB554,  // Entry 2
        0xAAAAB554,  // Entry 3
        0xAAAAB554,  // Entry 4
        0xAAAAB554,  // Entry 5
        0xAAAAB554,  // Entry 6
        0xAAAAB554   // Entry 7
    };
    
    for (uint8_t i = 0; i < 8; i++) {
        driver.motorControl.SetMicrostepLookupTable(i, lut_values[i]);
    }
    
    // Configure lookup table segmentation
    driver.motorControl.SetMicrostepLookupTableSegmentation(
        2, 2, 2, 2,  // Width selections (0-3)
        0, 0, 0      // Segment start positions
    );
    
    // Set start current
    driver.motorControl.SetMicrostepLookupTableStart(0);
}
```

**Note**: Custom lookup tables require deep understanding of motor control. Most users should use the default table.

## Position Comparison (X_COMPARE)

The X_COMPARE register generates a pulse when XACTUAL equals the compare value.

### Use Case

```cpp
void setupPositionPulse() {
    // Generate pulse when position reaches 1000 steps
    driver.rampControl.SetComparePosition(1000);
    
    // The pulse appears on SWP_DIAG1 output pin
    // Useful for triggering external events at specific positions
}
```

## Lost Steps Counter

When dcStep is enabled, the driver can count lost steps.

### Reading Lost Steps

```cpp
void checkStepLoss() {
    uint32_t lost_steps = driver.diagnostics.GetLostSteps();
    
    if (lost_steps > 0) {
        printf("Warning: %u steps lost\n", lost_steps);
        // Handle step loss (re-home, adjust current, etc.)
    }
}
```

**Note**: Lost steps counter only works when `SD_MODE=1` (external step/dir mode).

## Complete Advanced Setup Example

```cpp
#include "tmc5160.hpp"

void setupAdvancedMotor() {
    // Basic initialization
    tmc5160::DriverConfig cfg{};
    driver.Initialize(cfg);
    
    // Configure CoolStep
    tmc5160::CoolStepConfig coolstep{};
    coolstep.semin = 2;
    coolstep.semax = 5;
    coolstep.seup = 1;
    coolstep.sedn = 2;
    driver.motorControl.ConfigureCoolStep(coolstep);
    
    // Configure reference switches
    tmc5160::ReferenceSwitchConfig ref_switch{};
    ref_switch.stop_left_enable = true;
    ref_switch.stop_right_enable = true;
    ref_switch.en_softstop = true;
    driver.rampControl.ConfigureReferenceSwitch(ref_switch);
    
    // Set mode change speeds
    driver.motorControl.SetModeChangeSpeeds(
        1000.0f,  // stealthChop threshold
        500.0f,   // CoolStep threshold
        5000.0f   // High-speed threshold
    );
    
    // Enable motor
    driver.motorControl.Enable();
}
```

## Troubleshooting

### CoolStep Not Activating

**Problem**: Current doesn't reduce despite low load

**Solution**:
- Verify `TCOOLTHRS` is set correctly
- Check that motor speed is between `TCOOLTHRS` and `THIGH`
- Ensure StallGuard2 is properly configured
- Verify `semin` threshold is appropriate

### dcStep Causes Step Loss

**Problem**: Steps are lost when dcStep is active

**Solution**:
- Increase motor current
- Reduce `vdc_min` threshold
- Disable dcStep if precision is critical
- Check mechanical system for binding

### Reference Switches Not Working

**Problem**: Endstops don't trigger stop

**Solution**:
- Verify switch wiring and polarity
- Check `stop_left_enable` / `stop_right_enable` are set
- Verify switch signal reaches driver (check IO_INPUT register)
- Ensure `en_softstop` is enabled for smooth stopping

---

**Navigation**
⬅️ [Previous: Sensorless Homing](special_features_sensorless_homing.md) | [Next: Troubleshooting ➡️](troubleshooting.md) | [Docs Hub 📚](index.md)

