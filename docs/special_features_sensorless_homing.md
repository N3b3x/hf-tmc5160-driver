---
layout: default
title: "🏠 Sensorless Homing"
description: "Homing without endstops using StallGuard2"
nav_order: 11
parent: "📚 Documentation"
permalink: /docs/special_features_sensorless_homing/
---

# Sensorless Homing

The TMC51x0 (TMC5130 & TMC5160) supports sensorless homing using StallGuard2 stall detection. This allows you to home the motor without physical endstop switches by detecting when the motor stalls against a mechanical stop.

## Overview

Sensorless homing uses the StallGuard2 feature to detect when the motor stalls. When the motor hits a mechanical stop, the load increases, causing StallGuard2 to trigger. The driver can automatically stop the motor when a stall is detected.

## When to Use Sensorless Homing

- ✅ Applications without space for endstop switches
- ✅ Cost-sensitive designs
- ✅ Homing to a known mechanical stop
- ✅ When stall detection is acceptable (not too sensitive)
- ❌ Applications requiring precise homing position
- ❌ When mechanical stops aren't reliable
- ❌ High-speed homing applications

## Prerequisites

1. **Mechanical Stop**: A reliable physical stop that the motor can hit
2. **StallGuard2 Configuration**: Properly tuned stall threshold
3. **Motor Enabled**: Motor must be enabled before homing
4. **SpreadCycle Mode**: StallGuard2 ONLY works in SpreadCycle mode (`en_pwm_mode = 0`). It DOES NOT work in StealthChop mode.

## Important: StallGuard2 and StealthChop Compatibility

**StallGuard2 and StealthChop are mutually exclusive.**

- **StealthChop (`en_pwm_mode = 1`)**: Voltage-controlled mode for silent operation. The driver cannot measure back-EMF load accurately, so StallGuard2 values are invalid or zero.
- **SpreadCycle (`en_pwm_mode = 0`)**: Current-controlled mode for higher torque. The driver can measure load, enabling StallGuard2.

**To perform sensorless homing:**
1. Disable StealthChop (switch to SpreadCycle)
2. Perform the homing sequence
3. Re-enable StealthChop (if desired for normal operation)

**Note on Mode Switching**: Switching between StealthChop and SpreadCycle can cause a small jerk or "click" sound. This is normal. Ideally, perform the switch at standstill.

## Basic Usage

### Simple Homing Sequence

```cpp
#include "tmc51x0.hpp"

bool homeMotor() {
    // Configure StallGuard2 for homing (SGT threshold should be set once per motor)
    // This should be done during Initialize() or via ConfigureStallGuard() before homing
    tmc51x0::StallGuardConfig sg_config{};
    sg_config.threshold = -10;        // Stall threshold (tune for your motor)
    sg_config.enable_filter = true;     // Enable filter for stability
    
    if (!driver.diagnostics.ConfigureStallGuard(sg_config)) {
        return false;
    }
    
    // Perform homing in negative direction
    // Note: Uses existing SGT threshold from motor configuration
    // Automatically caches and restores settings (StealthChop, SW_MODE, ramp settings)
    int32_t home_position = 0;
    float search_speed = 0.01f;  // 0.01 rev/s (~0.6 RPM) - Unit::RevPerSec is default
    
    if (!driver.homing.PerformSensorlessHoming(
            false,  // false = negative direction
            search_speed,
            home_position)) {
        return false;
    }
    
    // Set current position as home (0)
    driver.rampControl.SetCurrentPosition(0);
    
    printf("Homed to position: %d\n", home_position);
    return true;
}
```

## Homing Methods

Homing methods are now in the `homing` subsystem and automatically cache/restore settings:

- **`driver.homing.PerformSensorlessHoming()`**: Uses existing StallGuard configuration (SGT threshold from motor config)
- **`driver.homing.PerformSwitchHoming()`**: Uses reference switches for homing

Both methods automatically:
- Cache current settings (StealthChop, SW_MODE, ramp settings)
- Disable StealthChop if enabled (StallGuard requires SpreadCycle)
- Restore cached settings after homing completes

**Note**: StealthChop is automatically disabled during sensorless homing (StallGuard requires SpreadCycle mode) and restored afterward.

## StallGuard2 Configuration

### Understanding Stall Threshold (`sgt`)

The `sgt` parameter controls stall sensitivity:
- **Range**: -64 to +63
- **Lower values** (more negative): More sensitive, detects stall earlier
- **Higher values** (more positive): Less sensitive, requires more load

### Tuning Stall Threshold

1. **Start with a conservative value**: `sgt = 0`
2. **Move motor toward stop slowly**
3. **Monitor StallGuard value**:

```cpp
void tuneStallThreshold() {
    // Move motor slowly
    driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_POS);
    driver.rampControl.SetMaxSpeed(100.0f);  // Slow speed
    driver.motorControl.Enable();
    
    // Monitor StallGuard value
    while (!driver.rampControl.IsTargetReached()) {
        uint16_t sg_value = driver.diagnostics.GetStallGuard();
        printf("StallGuard: %d\n", sg_value);
        
        // When motor hits stop, SG value will drop significantly
        if (sg_value < 100) {  // Threshold depends on your motor
            printf("Stall detected!\n");
            break;
        }
    }
    
    driver.rampControl.Stop();
}
```

3. **Set threshold**: Use a value slightly above the stall value
   - If stall occurs at SG=50, use `sgt = 30-40`
   - If stall occurs at SG=200, use `sgt = 150-180`

## Complete Homing Example

```cpp
#include "tmc51x0.hpp"

class MotorController {
private:
    tmc51x0::TMC51x0<Esp32SPI> &driver_;
    int8_t stall_threshold_;
    
public:
    MotorController(tmc51x0::TMC51x0<Esp32SPI> &driver) 
        : driver_(driver), stall_threshold_(-10) {}
    
    bool home() {
        // 1. Configure StallGuard2
        tmc51x0::StallGuardConfig sg_config{};
        sg_config.threshold = stall_threshold_;
        sg_config.enable_filter = true;  // Filter for stability
        
        if (!driver_.diagnostics.ConfigureStallGuard(sg_config)) {
            return false;
        }
        
        // 2. Enable StallGuard stop in SW_MODE
        // (PerformSensorlessHoming automatically caches and restores settings)
        
        // 3. Home in negative direction
        int32_t home_pos = 0;
        float search_speed = 0.01f;  // 0.01 rev/s (~0.6 RPM) - Unit::RevPerSec is default
        
        if (!driver_.homing.PerformSensorlessHoming(
                false,  // negative direction
                stall_threshold_,
                search_speed,
                home_pos)) {
            return false;
        }
        
        // 4. Set position to zero
        driver_.rampControl.SetCurrentPosition(0);
        
        return true;
    }
    
    bool moveToHome() {
        // Move to home position (0)
        driver_.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
        driver_.rampControl.SetTargetPosition(0);
        driver_.motorControl.Enable();
        
        // Wait for completion
        while (!driver_.rampControl.IsTargetReached()) {
            // Could add timeout here
        }
        
        return true;
    }
};
```

## Homing Sequence Best Practices

### 1. Pre-Home Setup

```cpp
// Ensure motor is enabled
driver.motorControl.Enable();

// Set appropriate speed (not too fast)
driver.rampControl.SetMaxSpeed(0.01f);  // 0.01 rev/s (~0.6 RPM) - Unit::RevPerSec is default

// Set acceleration
driver.rampControl.SetAcceleration(1000.0f);
```

### 2. Homing Direction

- **Negative direction**: Typically used for homing to "zero" position
- **Positive direction**: Can be used if mechanical stop is at max position
- Choose direction that moves away from work area

### 3. Post-Home Actions

```cpp
// After homing completes:
// 1. Set current position to zero
driver.rampControl.SetCurrentPosition(0);

// 2. Move away from stop slightly (optional)
driver.rampControl.SetTargetPosition(100);  // Move 100 steps away
while (!driver.rampControl.IsTargetReached()) {
    // Wait
}

// 3. Set new zero position
driver.rampControl.SetCurrentPosition(0);
```

## Troubleshooting

### Motor Doesn't Stop at Stall

**Problem**: Motor continues moving after hitting stop

**Solution**:
- Lower `sgt` value (more negative)
- Increase `sfilt` filter for stability
- Verify `sg_stop` is enabled in `SW_MODE`
- Check that StallGuard2 is properly configured

### False Stall Detection

**Problem**: Motor stops before reaching mechanical stop

**Solution**:
- Increase `sgt` value (less sensitive)
- Check for mechanical binding
- Verify motor current is sufficient
- Reduce acceleration to avoid false triggers

### Inconsistent Homing Position

**Problem**: Home position varies between homing cycles

**Solution**:
- Use `sfilt` filter for stable readings
- Ensure consistent mechanical stop
- Consider moving away from stop after homing
- Use reference switches for more precision

## Advanced: Manual Stall Detection

For more control, you can implement manual stall detection:

```cpp
bool homeManual() {
    // Configure StallGuard2
    tmc51x0::StallGuardConfig sg_config{};
    sg_config.threshold = -10;
    sg_config.enable_filter = true;
    driver.diagnostics.ConfigureStallGuard(sg_config);
    
    // Start movement
    driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_NEG);
    driver.rampControl.SetMaxSpeed(500.0f);
    driver.motorControl.Enable();
    
    // Monitor for stall
    const uint16_t STALL_THRESHOLD = 100;
    const uint32_t TIMEOUT_MS = 10000;
    uint32_t start_time = getCurrentTimeMs();
    
    while (getCurrentTimeMs() - start_time < TIMEOUT_MS) {
        uint16_t sg_value = driver.diagnostics.GetStallGuard();
        
        if (sg_value < STALL_THRESHOLD) {
            // Stall detected
            driver.rampControl.Stop();
            driver.rampControl.SetCurrentPosition(0);
            return true;
        }
    }
    
    // Timeout
    driver.rampControl.Stop();
    return false;
}
```

---

**Navigation**
⬅️ [Previous: Motor Setup](special_features_motor_setup.md) | [Next: Advanced Configuration ➡️](special_features_advanced_configuration.md) | [Docs Hub 📚](index.md)

