---
layout: default
title: "🔧 Advanced Configuration"
description: "Advanced features: CoolStep, dcStep, freewheeling, and more"
nav_order: 12
parent: "📚 Documentation"
permalink: /docs/special_features_advanced_configuration/
---

# Advanced Configuration

This guide covers advanced TMC51x0 (TMC5130 & TMC5160) features including CoolStep current reduction, dcStep automatic commutation, freewheeling modes, reference switches, and microstep lookup tables.

## Overview

The TMC51x0 (TMC5130 & TMC5160) provides several advanced features for optimizing motor performance:

- **CoolStep**: Automatically reduces current when load is low
- **dcStep**: Automatic commutation for DC motor-like operation
- **Freewheeling**: Low-power modes when motor is stationary
- **Reference Switches**: Endstop support for homing and limits
- **Microstep Lookup Tables**: Custom microstep interpolation

## CoolStep Current Reduction

CoolStep is an automatic smart energy optimization feature that dynamically adjusts motor current based on mechanical load, measured via StallGuard2. It can reduce power consumption by up to 75% while maintaining adequate torque reserve.

### Benefits

- **Energy Savings**: Up to 75% power reduction when load is low
- **Reduced Heat**: Lower current means less heat generation
- **Extended Motor Life**: Cooler operation increases motor lifespan
- **Cost Savings**: Smaller power supplies and less cooling infrastructure needed
- **Automatic Operation**: No manual intervention required once configured

### When to Use CoolStep

**✅ Recommended for:**
- Applications with varying load (e.g., CNC machines, 3D printers)
- Battery-powered systems (extends battery life)
- Motors that run hot under constant load
- Long-running applications (reduces cumulative power consumption)
- Applications where 30-50% torque reserve is acceptable

**❌ Not recommended for:**
- Constant high-load applications (minimal benefit)
- Applications requiring precise current control
- Systems where maximum torque must always be available
- Applications where load changes are too rapid for CoolStep to track

### Prerequisites

**Critical Requirements:**
1. **SpreadCycle Mode**: CoolStep requires SpreadCycle mode (`en_pwm_mode=0`). StealthChop must be disabled.
2. **StallGuard2 Tuning**: CoolStep uses StallGuard2 to measure load. You must first tune StallGuard2 threshold (`sgt`) for your motor.
3. **Stable Velocity Range**: CoolStep works best at constant velocities. Set velocity thresholds to match your typical operating speed.

### How CoolStep Works

CoolStep operates by continuously monitoring the StallGuard2 (SG) value, which represents motor load:

1. **Load Measurement**: StallGuard2 provides a 10-bit value (0-1023) where:
   - **Low SG value (0-100)**: High mechanical load (motor working hard)
   - **High SG value (500-1023)**: Low mechanical load (motor coasting)

2. **Current Adjustment**:
   - **When SG < Lower Threshold**: CoolStep increases current (load is increasing)
   - **When SG ≥ Upper Threshold**: CoolStep decreases current (load is decreasing)
   - **Between Thresholds**: Current remains stable (hysteresis prevents oscillation)

3. **Current Range**: CoolStep adjusts current between:
   - **Maximum**: IRUN (full current setting)
   - **Minimum**: 50% or 25% of IRUN (configurable via `min_current`)

4. **Velocity Limits**: CoolStep only operates between `min_velocity` and `max_velocity` thresholds.

### Understanding SG Thresholds

CoolStep thresholds are configured using register values (`semin` and `semax`) that map to actual StallGuard2 values:

**Threshold Calculation**:
- **Lower threshold** (SG units) = `semin * 32`
  - Example: `semin = 2` → threshold = 64 (when SG < 64, increase current)
- **Upper threshold** (SG units) = `(semin + semax + 1) * 32`
  - Example: `semin = 2`, `semax = 5` → threshold = 256 (when SG >= 256, decrease current)

**Converting from SG Thresholds to Register Values**:
- To set a lower threshold of 64 SG units: `semin = 64 / 32 = 2`
- To set an upper threshold of 256 SG units with `semin = 2`: `semax = (256 / 32) - 2 - 1 = 5`

**Configuration**:

**Note**: The library automatically converts SG thresholds to register values internally. You only need to specify the actual SG values (0-1023).

### Configuration Parameters

#### Threshold Configuration

**SEMIN (Lower Threshold Register)**:
- **Purpose**: Triggers current increase when load increases
- **Range**: 0-15 (0 = CoolStep disabled)
- **Typical Values**: 1-4 (thresholds 32-128 SG units)
- **Too Low**: Current increases too late, risk of stalling
- **Too High**: Current increases unnecessarily, reducing efficiency

**SEMAX (Hysteresis Register)**:
- **Purpose**: Controls the gap between lower and upper thresholds
- **Range**: 0-15
- **Typical Values**: 3-8 (provides hysteresis gap of 96-256 SG units)
- **Too Low**: Current decreases too aggressively, may cause stalling
- **Too High**: Current never decreases, no energy savings

**Hysteresis**: The gap between thresholds prevents oscillation. Typical gap is 3-5 SEMAX units (96-160 SG units).

#### Response Speed

**Increment Step (`increment_step`)**:
- `STEP_1`: Slow, smooth response (best for slowly varying loads)
- `STEP_2`: Moderate response (recommended default)
- `STEP_4`: Fast response (for rapidly increasing loads)
- `STEP_8`: Very fast response (may cause oscillations)

**Decrement Speed (`decrement_speed`)**:
- `EVERY_32`: Slowest reduction, most stable (recommended for smooth operation)
- `EVERY_8`: Moderate reduction (recommended default)
- `EVERY_2`: Fast reduction (for rapidly decreasing loads)
- `EVERY_1`: Fastest reduction (may cause oscillations)

**Rule of Thumb**: Increment should be faster than decrement to prevent stalling.

#### Minimum Current (`min_current`)

- `HALF_IRUN` (50%): Conservative, maintains more torque reserve
- `QUARTER_IRUN` (25%): Aggressive, maximum energy savings

**Recommendation**: Start with `HALF_IRUN`, switch to `QUARTER_IRUN` if you need more savings and can tolerate lower torque.

#### Filter (`enable_filter`)

- **Enabled**: StallGuard2 updates every 4 fullsteps (smoother, slower response)
- **Disabled**: StallGuard2 updates every fullstep (faster response, more sensitive)

**Recommendation**: Enable filter for smoother operation unless you need fastest response.

#### Velocity Thresholds

**Purpose**: Define the speed range where CoolStep is active.

**`min_velocity`**: CoolStep disabled below this speed
- Set to match the lowest speed where StallGuard2 gives stable readings
- Typical: 0.004-0.02 rev/s (~0.24-1.2 RPM) depending on motor

**`max_velocity`**: CoolStep disabled above this speed
- Set to match the highest speed where StallGuard2 is reliable
- Typical: 0.1-0.4 rev/s (~6-24 RPM) depending on motor
- Set equal to `min_velocity` to disable CoolStep during acceleration/deceleration

**Best Practice**: Set thresholds to match your typical constant-velocity operating range.

### Configuration Examples

#### Basic Configuration

```cpp
#include "tmc51x0.hpp"

void configureCoolStep() {
    // IMPORTANT: CoolStep requires SpreadCycle mode
    driver.motorControl.SetStealthChopEnabled(false);
    
    // Configure CoolStep with user-friendly API
    tmc51x0::CoolStepConfig coolstep{};
    
    // Set thresholds using actual SG values (0-1023)
    // Automatically converted to register values internally
    coolstep.lower_threshold_sg = 64;   // Increase current when SG < 64
    coolstep.upper_threshold_sg = 256;  // Decrease current when SG >= 256
    
    // Moderate response speeds
    coolstep.increment_step = tmc51x0::CoolStepIncrementStep::STEP_2;
    coolstep.decrement_speed = tmc51x0::CoolStepDecrementSpeed::EVERY_8;
    
    // Conservative minimum current
    coolstep.min_current = tmc51x0::CoolStepMinCurrent::HALF_IRUN;
    
    // Enable filter for smoother operation
    coolstep.enable_filter = true;
    
    // Set velocity thresholds (CoolStep active between these speeds)
    coolstep.min_velocity = 0.01f;    // Enable above 0.01 rev/s (~0.6 RPM)
    coolstep.max_velocity = 0.1f;     // Disable above 0.1 rev/s (~6 RPM)
    coolstep.velocity_unit = tmc51x0::Unit::RevPerSec;  // Recommended default
    
    // Configure CoolStep (automatically sets velocity thresholds)
    driver.motorControl.ConfigureCoolStep(coolstep);
}
```

#### Quick Setup with Helper Constructor

```cpp
// Quick setup with common defaults
tmc51x0::CoolStepConfig coolstep(64, 256, 0.01f, 0.1f, tmc51x0::Unit::RevPerSec);
coolstep.increment_step = tmc51x0::CoolStepIncrementStep::STEP_2;
coolstep.decrement_speed = tmc51x0::CoolStepDecrementSpeed::EVERY_8;
driver.motorControl.ConfigureCoolStep(coolstep);
```

#### Aggressive Energy Savings

```cpp
// Maximum energy savings configuration
tmc51x0::CoolStepConfig coolstep{};
coolstep.lower_threshold_sg = 96;   // Higher threshold = more aggressive reduction
coolstep.upper_threshold_sg = 320;  // Wider gap for stability (automatically calculates SEMAX)
coolstep.increment_step = tmc51x0::CoolStepIncrementStep::STEP_4;  // Fast increase
coolstep.decrement_speed = tmc51x0::CoolStepDecrementSpeed::EVERY_32;  // Slow decrease
coolstep.min_current = tmc51x0::CoolStepMinCurrent::QUARTER_IRUN;  // 25% minimum
coolstep.enable_filter = true;
coolstep.min_velocity = 10.0f;
coolstep.max_velocity = 120.0f;
coolstep.velocity_unit = tmc51x0::Unit::RPM;
driver.motorControl.ConfigureCoolStep(coolstep);
```

#### Fast Response Configuration

```cpp
// For rapidly varying loads
tmc51x0::CoolStepConfig coolstep{};
coolstep.lower_threshold_sg = 48;   // Lower threshold = faster response
coolstep.upper_threshold_sg = 192;  // Smaller gap = faster adjustment (automatically calculates SEMAX)
coolstep.increment_step = tmc51x0::CoolStepIncrementStep::STEP_8;  // Very fast increase
coolstep.decrement_speed = tmc51x0::CoolStepDecrementSpeed::EVERY_2;  // Fast decrease
coolstep.min_current = tmc51x0::CoolStepMinCurrent::HALF_IRUN;
coolstep.enable_filter = false;  // Disable filter for fastest response
coolstep.min_velocity = 5.0f;
coolstep.max_velocity = 100.0f;
coolstep.velocity_unit = tmc51x0::Unit::RPM;
driver.motorControl.ConfigureCoolStep(coolstep);
```

### Tuning Guide

#### Step 1: Tune StallGuard2 First

Before configuring CoolStep, tune StallGuard2 threshold for your motor:

```cpp
// Configure StallGuard2
tmc51x0::StallGuardConfig sg_config{};
sg_config.threshold = 0;  // Start with 0, adjust based on your motor
sg_config.enable_filter = true;  // Enable filter for stable readings
driver.stallGuard.ConfigureStallGuard(sg_config);

// Monitor SG_RESULT during operation
uint16_t sg_value = driver.stallGuard.GetStallGuardResult().Value();
// Adjust threshold until SG_RESULT is in the middle of the range (200-600) at typical load
```

#### Step 2: Determine Operating SG Range

Monitor SG_RESULT during normal operation:

```cpp
// Run motor at typical load and velocity
uint16_t sg_min = 1023;
uint16_t sg_max = 0;

for (int i = 0; i < 1000; i++) {
    uint16_t sg = driver.stallGuard.GetStallGuardResult().Value();
    sg_min = std::min(sg_min, sg);
    sg_max = std::max(sg_max, sg);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Use these values to calculate register thresholds
// Lower threshold: sg_min + 20% margin
// Upper threshold: sg_max - 20% margin
```

#### Step 3: Configure CoolStep Thresholds

Calculate and set register values based on your measured SG range:

```cpp
// Calculate thresholds in SG units
uint16_t lower_sg = sg_min + (sg_max - sg_min) * 0.2;  // 20% above minimum
uint16_t upper_sg = sg_max - (sg_max - sg_min) * 0.2;  // 20% below maximum

// Set thresholds directly (register values calculated automatically)
coolstep.lower_threshold_sg = lower_sg;  // Automatically converts to SEMIN internally
coolstep.upper_threshold_sg = upper_sg;  // Automatically converts to SEMAX internally
```

#### Step 4: Test and Adjust

1. **If motor stalls**: Lower `lower_threshold_sg` or increase `increment_step`
2. **If no energy savings**: Raise `upper_threshold_sg` or increase `decrement_speed`
3. **If oscillations**: Increase gap between thresholds or enable filter
4. **If response too slow**: Increase `increment_step` or decrease `decrement_speed`

### Best Practices

1. **Start Conservative**: Begin with moderate settings and gradually optimize
2. **Monitor SG Values**: Use `GetStallGuardResult()` to understand your motor's load profile
3. **Match Velocity Range**: Set velocity thresholds to your typical operating speeds
4. **Test Under Load**: Test CoolStep under actual operating conditions
5. **Maintain Torque Reserve**: Ensure minimum current provides adequate torque (30-50% reserve)
6. **Disable During Acceleration**: Set `min_velocity = max_velocity` to disable during accel/decel

### Troubleshooting

**Problem**: Motor stalls when load increases
- **Solution**: Lower `lower_threshold_sg` or increase `increment_step`

**Problem**: No energy savings observed
- **Solution**: Raise `upper_threshold_sg`, check that velocity thresholds are correct

**Problem**: Current oscillates rapidly
- **Solution**: Increase gap between thresholds, enable filter, or slow down `decrement_speed`

**Problem**: CoolStep doesn't activate
- **Solution**: Check that SpreadCycle is enabled, verify velocity thresholds, ensure `semin > 0`

**Problem**: Response too slow
- **Solution**: Increase `increment_step`, disable filter, reduce `decrement_speed`

### Technical Details

**Register Mapping**:
- `SEMIN` (bits 3-0): Lower threshold = SEMIN × 32
- `SEMAX` (bits 11-8): Upper threshold = (SEMIN + SEMAX + 1) × 32
- `SEUP` (bits 6-5): Increment step (0=1, 1=2, 2=4, 3=8)
- `SEDN` (bits 14-13): Decrement speed (0=32, 1=8, 2=2, 3=1 measurements)
- `SEIMIN` (bit 15): Minimum current (0=50%, 1=25% of IRUN)
- `SFILT` (bit 24): Filter enable (0=every step, 1=every 4 steps)

**Current Scale**: CoolStep adjusts `CSACTUAL` register (0-31), which scales IRUN:
- CSACTUAL = 31: Full IRUN current
- CSACTUAL = 16: 50% IRUN (if SEIMIN=0)
- CSACTUAL = 8: 25% IRUN (if SEIMIN=1)

**Note**: CoolStep requires StallGuard2, which ONLY works in SpreadCycle mode (`en_pwm_mode=0`). CoolStep is automatically disabled if StealthChop is enabled.

## DcStep Automatic Commutation

DcStep is an automatic commutation mode that allows the motor to run near its load limit without losing steps. When the motor becomes overloaded, it automatically slows down to a velocity where it can still drive the load, preventing stalls.

### Benefits

- **No Step Loss**: Motor never loses steps in overload conditions
- **Maximum Performance**: Application works as fast as possible
- **Automatic Acceleration**: Highest possible acceleration automatically
- **Energy Efficient**: Highest energy efficiency at speed limit
- **Full Torque**: Uses fullstep drive for maximum motor torque
- **Cost Effective**: Cheaper motors can be used

### When to Use DcStep

**✅ Recommended for:**
- Applications with varying mechanical load
- Applications where maximum velocity is limited by load
- Systems that need to handle overload conditions gracefully
- Applications where step loss prevention is critical
- High-torque applications requiring fullstep operation

**❌ Not recommended for:**
- High-precision positioning applications (step loss may occur)
- Applications requiring constant velocity regardless of load
- Systems where step loss is completely unacceptable
- Applications with very light, constant load

### Prerequisites

**Critical Requirements:**
1. **SD_MODE**: DcStep can be enabled via VDCMIN threshold (internal ramp generator) or via DCEN pin (external step/dir mode, SD_MODE=1)
2. **CHOPCONF Settings**: `vhighfs` and `vhighchm` must be set to 1 (handled automatically)
3. **TOFF Setting**: Should be >2, preferably 8-15 for DcStep operation
4. **Microstep LUT**: Default microstep lookup table must be used (phase polarity requirements)

### How DcStep Works

1. **Load Monitoring**: Continuously monitors motor load via PWM on-time measurement
2. **Velocity Adjustment**: When overloaded, automatically reduces velocity to maintain torque
3. **Fullstep Operation**: Operates in fullstep mode for maximum torque
4. **Stall Detection**: Can detect stalls and stop motor if configured

### Configuration Parameters

#### Minimum Velocity (`min_velocity`)

**Purpose**: Lower velocity threshold for DcStep activation.

- Below this velocity: Motor operates in normal microstep mode
- Above this velocity: DcStep is active
- In DcStep: Motor operates at minimum this velocity even when blocked

**Setting**: Set to the lowest operating velocity where DcStep gives reliable detection.

**Typical Range**: 0.004-0.04 rev/s (~0.24-2.4 RPM) depending on motor and application.

#### PWM On-Time (`pwm_on_time_us`)

**Purpose**: Controls reference pulse width for DcStep load measurement.

**Effect**:
- **Higher value**: Higher torque capability, higher velocity capability
- **Lower value**: Operation down to lower velocity (as set by min_velocity)

**Setting**: Should be set slightly above effective blank time (TBL from CHOPCONF).

**Auto-calculation**: If set to 0, automatically calculated from blank time (TBL + 20 clock cycles).

**Typical Range**: 1-10µs (for 12MHz clock) or 0.5-5µs (for 24MHz clock).

**Blank Time Reference**:
- TBL=0: 16 clock cycles (~1.33µs @ 12MHz)
- TBL=1: 24 clock cycles (~2.0µs @ 12MHz)
- TBL=2: 36 clock cycles (~3.0µs @ 12MHz, typical)
- TBL=3: 54 clock cycles (~4.5µs @ 12MHz)

#### Stall Detection (`stall_sensitivity`)

**Purpose**: Controls stall detection threshold in DcStep mode.

**Options**:
- `DISABLED`: No stall detection (dc_sg = 0)
- `LOW`: Low sensitivity - fewer false positives (dc_sg ≈ dc_time / 20)
- `MODERATE`: Moderate sensitivity - balanced (dc_sg ≈ dc_time / 16, recommended)
- `HIGH`: High sensitivity - detects stalls earlier (dc_sg ≈ dc_time / 12)

**Auto-calculation**: Automatically calculated from `pwm_on_time_us` based on sensitivity level.

#### Stop on Stall (`stop_on_stall`)

**Purpose**: Stop motor when stall is detected.

- If `true`: Motor stops (VACTUAL = 0) when stall detected
- Motor remains stopped until `RAMP_STAT.event_stop_sg` flag is read

**Note**: Requires `stall_sensitivity != DISABLED` and StallGuard2 configured (TCOOLTHRS set).

### Configuration Examples

#### Basic Configuration

```cpp
#include "tmc51x0.hpp"

void configureDcStep() {
    // Configure DcStep with user-friendly API
    tmc51x0::DcStepConfig dcstep{};
    
    // Set minimum velocity threshold (with unit support)
    dcstep.min_velocity = 0.02f;      // Enable DcStep above 0.02 rev/s (~1.2 RPM)
    dcstep.velocity_unit = tmc51x0::Unit::RevPerSec;  // Recommended default
    
    // Auto-calculate PWM on-time from blank time (recommended)
    dcstep.pwm_on_time_us = 0.0f;  // 0 = auto-calculate
    
    // Moderate stall detection sensitivity (recommended)
    dcstep.stall_sensitivity = tmc51x0::DcStepStallSensitivity::MODERATE;
    
    // Don't stop on stall (continue operation)
    dcstep.stop_on_stall = false;
    
    // Configure DcStep
    driver.motorControl.ConfigureDcStep(dcstep);
    
    // Note: DcStep automatically sets CHOPCONF.vhighfs and CHOPCONF.vhighchm
    // Note: Ensure CHOPCONF.TOFF > 2 (preferably 8-15) for DcStep operation
}
```

#### Quick Setup with Helper Constructor

```cpp
// Quick setup with common defaults
tmc51x0::DcStepConfig dcstep(0.02f, tmc51x0::Unit::RevPerSec);
// Auto-calculates PWM on-time, uses MODERATE sensitivity
driver.motorControl.ConfigureDcStep(dcstep);
```

#### Custom PWM On-Time Configuration

```cpp
// For applications requiring specific PWM on-time
tmc51x0::DcStepConfig dcstep{};
dcstep.min_velocity = 60.0f;
dcstep.velocity_unit = tmc51x0::Unit::RPM;
dcstep.pwm_on_time_us = 3.0f;  // 3µs PWM on-time (for 12MHz clock = 36 clock cycles)
dcstep.stall_sensitivity = tmc51x0::DcStepStallSensitivity::HIGH;  // High sensitivity
dcstep.stop_on_stall = true;  // Stop motor on stall
driver.motorControl.ConfigureDcStep(dcstep);
```

#### High-Torque Configuration

```cpp
// Maximum torque capability
tmc51x0::DcStepConfig dcstep{};
dcstep.min_velocity = 120.0f;  // Higher threshold = more torque headroom
dcstep.velocity_unit = tmc51x0::Unit::RPM;
dcstep.pwm_on_time_us = 5.0f;  // Higher PWM on-time = higher torque capability
dcstep.stall_sensitivity = tmc51x0::DcStepStallSensitivity::LOW;  // Fewer false positives
dcstep.stop_on_stall = false;
driver.motorControl.ConfigureDcStep(dcstep);
```

#### Low-Velocity Operation

```cpp
// Operation down to low velocities
tmc51x0::DcStepConfig dcstep{};
dcstep.min_velocity = 30.0f;  // Lower threshold = operation at lower velocities
dcstep.velocity_unit = tmc51x0::Unit::RPM;
dcstep.pwm_on_time_us = 1.5f;  // Lower PWM on-time = operation at lower velocities
dcstep.stall_sensitivity = tmc51x0::DcStepStallSensitivity::MODERATE;
dcstep.stop_on_stall = true;  // Stop on stall for safety
driver.motorControl.ConfigureDcStep(dcstep);
```

### Tuning Guide

#### Step 1: Set Minimum Velocity

Set `min_velocity` to match your typical operating speed range:

```cpp
// Monitor actual motor velocity during operation
// Set threshold to lowest speed where DcStep should be active
dcstep.min_velocity = 0.01f;  // Example: Enable DcStep above 0.01 rev/s (~0.6 RPM)
```

#### Step 2: Configure PWM On-Time

**Option A: Auto-calculate (Recommended)**
```cpp
dcstep.pwm_on_time_us = 0.0f;  // Auto-calculates from blank time
```

**Option B: Manual tuning**
```cpp
// Start with value slightly above blank time
// TBL=2 → 36 clocks → ~3.0µs @ 12MHz
dcstep.pwm_on_time_us = 3.5f;  // Slightly above blank time

// Test under load:
// - If motor stalls: Increase pwm_on_time_us
// - If no benefit: Decrease pwm_on_time_us
```

#### Step 3: Configure Stall Detection

```cpp
// Start with MODERATE sensitivity
dcstep.stall_sensitivity = tmc51x0::DcStepStallSensitivity::MODERATE;

// Adjust based on results:
// - False positives: Use LOW sensitivity
// - Missed stalls: Use HIGH sensitivity
// - No stall detection needed: Use DISABLED
```

#### Step 4: Test and Adjust

1. **If motor stalls**: Increase `pwm_on_time_us` or increase `min_velocity`
2. **If no benefit**: Decrease `pwm_on_time_us` or check that load varies
3. **If false stall detection**: Decrease `stall_sensitivity` or disable it
4. **If missed stalls**: Increase `stall_sensitivity`

### Best Practices

1. **Start Conservative**: Begin with auto-calculated PWM on-time and moderate sensitivity
2. **Test Under Load**: Test DcStep under actual operating conditions
3. **Monitor Lost Steps**: Use `GetLostSteps()` to monitor step loss
4. **Match Velocity Range**: Set `min_velocity` to match your typical operating speeds
5. **Optimize PWM On-Time**: Tune `pwm_on_time_us` for your specific motor and load
6. **Use Appropriate TOFF**: Ensure CHOPCONF.TOFF > 2 (preferably 8-15) for DcStep

### Technical Details

**Register Mapping**:
- `VDCMIN` (bits 22..8): Minimum velocity threshold (lower 8 bits ignored)
- `DC_TIME` (bits 9..0): PWM on-time limit in clock cycles (0-1023)
- `DC_SG` (bits 23..16): Stall detection threshold (0-255, 0 = disabled)

**Conversion Formulas**:
- PWM on-time (clock cycles) = `(pwm_on_time_us * f_clk) / 1e6`
- DC_SG (auto-calculated) = `DC_TIME / divisor` (divisor: 20=LOW, 16=MODERATE, 12=HIGH)

**Blank Time Reference** (for auto-calculation):
- TBL=0: 16 clocks (~1.33µs @ 12MHz)
- TBL=1: 24 clocks (~2.0µs @ 12MHz)
- TBL=2: 36 clocks (~3.0µs @ 12MHz, typical)
- TBL=3: 54 clocks (~4.5µs @ 12MHz)

**Note**:
- In **internal ramp mode (SD_MODE=0)**, DcStep can be enabled above the VDCMIN threshold.
- In **external STEP/DIR mode (SD_MODE=1)**, DcStep is enabled via the external DCEN pin; VDCMIN does not enable DcStep in this mode.

DcStep automatically sets CHOPCONF.vhighfs and CHOPCONF.vhighchm to 1.

## StallGuard2 Load Measurement

StallGuard2 provides accurate measurement of motor load and can detect stalls. It's used for sensorless homing, CoolStep load-adaptive current reduction, and diagnostics.

### Benefits

- **Load Measurement**: Accurate measurement of mechanical load on motor
- **Stall Detection**: Detect motor stalls automatically
- **Sensorless Homing**: Enable homing without endstops
- **CoolStep Integration**: Provides load data for automatic current reduction
- **Diagnostics**: Monitor motor health and mechanical system condition

### When to Use StallGuard2

**✅ Recommended for:**
- Sensorless homing applications
- CoolStep load-adaptive current reduction
- Motor diagnostics and monitoring
- Applications requiring stall detection
- Systems where load monitoring is beneficial

**❌ Not recommended for:**
- StealthChop mode (StallGuard2 only works in SpreadCycle)
- Very low velocities (< 1 RPS for many motors)
- Very high velocities (where back EMF approaches supply voltage)

### Prerequisites

**Critical Requirements:**
1. **SpreadCycle Mode**: StallGuard2 requires SpreadCycle mode (`en_pwm_mode=0`). StealthChop must be disabled.
2. **Velocity Range**: StallGuard2 works best at moderate velocities (typically 1-5 RPS for smaller motors).
3. **Current Setting**: Best results at 30-70% of nominal motor current.

### How StallGuard2 Works

1. **Load Measurement**: Continuously measures motor load via back EMF and current phase relationship
2. **SG_RESULT Value**: Provides 10-bit value (0-1023) where:
   - **Low value (0-100)**: High mechanical load (motor working hard, near stall)
   - **High value (500-1023)**: Low mechanical load (motor coasting)
3. **Stall Detection**: When SG_RESULT reaches zero, motor is stalled or about to stall
4. **Filter Option**: Filter reduces measurement rate to one per electrical period (4 fullsteps) for smoother readings

### Configuration Parameters

#### Threshold (`threshold`)

**Purpose**: Controls StallGuard2 sensitivity for stall detection and sets optimum measurement range.

**Range**: -64 to +63 (signed 7-bit)

**Sensitivity Levels**:
- **Lower values (-64 to 0)**: Higher sensitivity (detects stalls easier, more false positives)
- **Higher values (0 to +63)**: Lower sensitivity (requires more torque to detect stall, fewer false positives)
- **Zero (0)**: Starting value, works with most motors (recommended starting point)

**Tuning Goal**: Adjust until SG_RESULT is between 0-100 at maximum load before stall.

**Typical Values**:
- Free-rotating motors: 0 to +20 (less sensitive, fewer false positives)
- Motors with mechanical stops: -20 to 0 (more sensitive for homing)
- High-torque applications: +10 to +30 (less sensitive)

#### Filter (`enable_filter`)

**Purpose**: Reduces measurement rate for smoother, more precise readings.

- **Enabled**: One measurement per electrical period (4 fullsteps) - smoother, compensates for motor construction variations
- **Disabled**: One measurement per fullstep - faster response, higher time resolution

**Recommendation**: 
- Enable for CoolStep (smoother readings)
- Disable for sensorless homing (faster response)

#### Velocity Thresholds

**Purpose**: Define the speed range where StallGuard2 operates reliably.

**`min_velocity`**: StallGuard2 disabled below this speed
- Set to match the lowest speed where StallGuard2 gives stable readings
- Typical: 200-1000 steps/s depending on motor
- For sensorless homing: Set to match search speed

**`max_velocity`**: StallGuard2 may not operate reliably above this speed
- Set to match the highest speed where StallGuard2 is reliable
- Typical: 5000-20000 steps/s depending on motor
- Set to 0 for no upper limit

**Best Practice**: Set thresholds to match your typical operating velocity range.

#### Stop on Stall (`stop_on_stall`)

**Purpose**: Stop motor automatically when stall is detected.

- If `true`: Motor stops (VACTUAL = 0) when stall detected
- Motor remains stopped until `RAMP_STAT.event_stop_sg` flag is read
- Motor can be restarted by reading/writing RAMP_STAT or disabling `sg_stop`

**Note**: Requires `min_velocity` to be set (TCOOLTHRS must be configured).

### Configuration Examples

#### Basic Configuration

```cpp
#include "tmc51x0.hpp"

void configureStallGuard() {
    // IMPORTANT: Enable SpreadCycle mode first
    driver.motorControl.SetStealthChopEnabled(false);
    
    // Configure StallGuard2 with user-friendly API
    tmc51x0::StallGuardConfig sg_config{};
    
    // Set threshold (starting value, works with most motors)
    sg_config.threshold = 0;
    
    // Enable filter for smoother readings
    sg_config.enable_filter = true;
    
    // Set velocity thresholds
    sg_config.min_velocity = 0.01f;    // Enable above 0.01 rev/s (~0.6 RPM)
    sg_config.max_velocity = 0.1f;     // Disable above 0.1 rev/s (~6 RPM)
    sg_config.velocity_unit = tmc51x0::Unit::RevPerSec;  // Recommended default
    
    // Don't stop on stall (for diagnostics/monitoring)
    sg_config.stop_on_stall = false;
    
    // Configure StallGuard2
    driver.stallGuard.ConfigureStallGuard(sg_config);
}
```

#### Quick Setup with Sensitivity Enum

```cpp
// Use sensitivity enum for convenience
tmc51x0::StallGuardConfig sg_config(
    tmc51x0::StallGuardSensitivity::MODERATE,  // Threshold = 0
    true,   // Enable filter
    10.0f,  // Min velocity
    120.0f, // Max velocity
    tmc51x0::Unit::RPM,
    false   // Don't stop on stall
);
driver.stallGuard.ConfigureStallGuard(sg_config);
```

#### Sensorless Homing Configuration

```cpp
// Configuration optimized for sensorless homing
tmc51x0::StallGuardConfig sg_config{};
sg_config.threshold = -10;  // More sensitive for stall detection
sg_config.enable_filter = false;  // Disable filter for faster response
sg_config.min_velocity = 10.0f;  // Match search speed
sg_config.max_velocity = 0.0f;  // No upper limit
sg_config.velocity_unit = tmc51x0::Unit::RPM;
sg_config.stop_on_stall = true;  // Stop motor on stall
driver.stallGuard.ConfigureStallGuard(sg_config);
```

#### CoolStep Integration Configuration

```cpp
// Configuration optimized for CoolStep
tmc51x0::StallGuardConfig sg_config{};
sg_config.threshold = 0;  // Moderate sensitivity
sg_config.enable_filter = true;  // Enable filter for smoother readings
sg_config.min_velocity = 10.0f;   // Match CoolStep min_velocity
sg_config.max_velocity = 120.0f;  // Match CoolStep max_velocity
sg_config.velocity_unit = tmc51x0::Unit::RPM;
sg_config.stop_on_stall = false;  // CoolStep handles current, not stopping
driver.stallGuard.ConfigureStallGuard(sg_config);
```

### Automatic Tuning with Comprehensive Velocity Range Analysis

The library provides an advanced automatic tuning function that prioritizes target velocity and provides comprehensive velocity range analysis:

```cpp
// Comprehensive tuning with velocity range analysis
tmc51x0::StallGuardTuningResult result;
float target_velocity = 0.6f;      // Most important - optimal SGT determined here (0.6 rev/s ≈ 36 RPM)
float min_velocity = 0.18f;         // Used to determine SGT range (30% of target)
float max_velocity = 0.9f;          // Used to determine SGT range (150% of target)

// Using AutoTuneStallGuard (recommended) with current reduction
// Note: Separate unit parameters for velocity and acceleration
bool success = driver.tuning.AutoTuneStallGuard(
    target_velocity, result,               // Target velocity: e.g., 0.6 rev/s (~36 RPM)
    0, 63,                                 // SGT search range
    0.06f,                                 // Acceleration: 0.06 rev/s²
    min_velocity, max_velocity,            // Velocity range: e.g., 0.18-0.9 rev/s
    tmc51x0::Unit::RevPerSec,              // Velocity unit (default, can be omitted)
    tmc51x0::Unit::RevPerSec,              // Acceleration unit (default, can be omitted)
    0.3f                                   // Current reduction factor: 30% (recommended)
);

if (success) {
    // Use optimal SGT at target velocity
    tmc51x0::StallGuardConfig sg_config;
    sg_config.threshold = result.optimal_sgt;
    sg_config.min_velocity = result.min_velocity_success ? min_velocity : result.actual_min_velocity;
    sg_config.max_velocity = result.max_velocity_success ? max_velocity : result.actual_max_velocity;
    driver.stallGuard.ConfigureStallGuard(sg_config);
    
    ESP_LOGI(TAG, "Optimal SGT: %d (SG_RESULT=%u at target)", 
             result.optimal_sgt, result.target_velocity_sg_result);
    
    if (!result.min_velocity_success) {
        ESP_LOGW(TAG, "Requested min velocity %.2f not achievable. Use %.2f instead.",
                 min_velocity, result.actual_min_velocity);
    }
    
    if (!result.max_velocity_success) {
        ESP_LOGW(TAG, "Requested max velocity %.2f not achievable. Use %.2f instead.",
                 max_velocity, result.actual_max_velocity);
    }
}
```

**Key Features**:
- **Target Velocity Priority**: Optimal SGT is determined at target velocity (most important parameter)
- **Velocity Range Analysis**: Tests min/max velocities to determine usable SGT range
- **Automatic Fallback**: If requested velocities don't work, finds actual achievable velocities
- **Comprehensive Results**: Returns detailed information about SGT values and velocity compatibility

**Result Structure (`StallGuardTuningResult`)**:
- `optimal_sgt`: Best SGT value at target velocity (primary result)
- `tuning_success`: Whether optimal SGT was found
- `min_velocity_success` / `max_velocity_success`: Whether requested velocities work
- `actual_min_velocity` / `actual_max_velocity`: Actual velocities that work (if requested ones don't)
- `target_velocity_sg_result` / `min_velocity_sg_result` / `max_velocity_sg_result`: SG_RESULT values at each velocity

### Tuning Guide

#### Step 1: Initial Setup

1. **Enable SpreadCycle**: Ensure StealthChop is disabled
2. **Set Velocity Thresholds**: Set `min_velocity` to match your typical operating speed
3. **Start with Default**: Set `threshold = 0` (starting value)

#### Step 2: Automatic Tuning (Recommended)

Use the comprehensive automatic tuning function for best results:

```cpp
tmc51x0::StallGuardTuningResult result;
// Using AutoTuneStallGuard (recommended) - includes current reduction
// Note: Separate unit parameters for velocity and acceleration
bool success = driver.tuning.AutoTuneStallGuard(
    target_velocity, result,               // Your target operating velocity: e.g., 0.6 rev/s
    0, 63,                                 // SGT search range
    0.06f,                                 // Acceleration: 0.06 rev/s²
    min_velocity, max_velocity, // Your velocity range: e.g., 0.18-0.9 rev/s
    tmc51x0::Unit::RevPerSec,  // Unit (default, can be omitted)
    0.3f                        // Current reduction factor: 30% (recommended)
);
```

#### Step 3: Interactive Tuning (Manual Alternative)

```cpp
// Operate motor at normal velocity and monitor SG_RESULT
uint16_t sg_min = 1023;
uint16_t sg_max = 0;

for (int i = 0; i < 1000; i++) {
    uint16_t sg = driver.stallGuard.GetStallGuardResult().Value();
    sg_min = std::min(sg_min, sg);
    sg_max = std::max(sg_max, sg);
    vTaskDelay(pdMS_TO_TICKS(10));
}

ESP_LOGI(TAG, "SG_RESULT range: %d to %d", sg_min, sg_max);
```

#### Step 3: Load Testing

```cpp
// Apply slowly increasing load
// If motor stalls before SG_RESULT reaches zero: decrease threshold
// If SG_RESULT reaches zero before motor stalls: increase threshold

// Goal: SG_RESULT between 0-100 at maximum load before stall
sg_config.threshold = /* adjusted value */;
driver.stallGuard.ConfigureStallGuard(sg_config);
```

#### Step 4: Fine-Tuning

1. **If false stalls**: Increase `threshold` (less sensitive)
2. **If missed stalls**: Decrease `threshold` (more sensitive)
3. **If readings noisy**: Enable `enable_filter`
4. **If response too slow**: Disable `enable_filter`

### Best Practices

1. **Use Automatic Tuning**: Use `TuneStallGuard()` with `StallGuardTuningResult` for comprehensive analysis
2. **Prioritize Target Velocity**: Set target velocity to your most common operating speed - this gets the best SGT value
3. **Specify Velocity Range**: Provide min/max velocities to determine the usable SGT range
4. **Check Results**: Always check `actual_min_velocity` and `actual_max_velocity` if requested velocities don't work
5. **Tune First**: Always tune StallGuard2 before using CoolStep
6. **Test Under Load**: Test under actual operating conditions
7. **Monitor SG_RESULT**: Use `GetStallGuardResult()` to understand your motor's load profile
8. **Match Velocity Range**: Set velocity thresholds to your typical operating speeds
9. **Use Filter Appropriately**: Enable for CoolStep, disable for sensorless homing
10. **Verify SpreadCycle**: Always ensure SpreadCycle is enabled before configuring

### Troubleshooting

**Problem**: SG_RESULT always reads 0
- **Solution**: Check that SpreadCycle is enabled (StealthChop disabled)

**Problem**: False stall detection
- **Solution**: Increase `threshold` value (less sensitive)

**Problem**: Missed stalls
- **Solution**: Decrease `threshold` value (more sensitive)

**Problem**: Noisy readings
- **Solution**: Enable `enable_filter` for smoother readings

**Problem**: Response too slow
- **Solution**: Disable `enable_filter` for faster response

**Problem**: StallGuard2 doesn't activate
- **Solution**: Check that `min_velocity` is set correctly (TCOOLTHRS configured)

### Technical Details

**Register Mapping**:
- `SGT` (bits 22..16): StallGuard2 threshold (-64 to +63, signed 7-bit)
- `SFILT` (bit 24): Filter enable (0=every step, 1=every 4 steps)
- `TCOOLTHRS`: Lower velocity threshold (bits 22..8 used for comparison)
- `THIGH`: Upper velocity threshold
- `SW_MODE.sg_stop` (bit 7): Stop on stall enable

**SG_RESULT Range**: 0-1023 (10-bit value)
- 0: Highest load (motor stalled or about to stall)
- 100-200: High load (motor working hard)
- 200-600: Moderate load (typical operating range)
- 600-1023: Low load (motor coasting)

**Measurement Update Rate**:
- Without filter: Every fullstep
- With filter: Every electrical period (4 fullsteps)

**Note**: StallGuard2 requires SpreadCycle mode (`en_pwm_mode=0`). StallGuard2 is not available in StealthChop mode.

## StealthChop Voltage PWM Mode

StealthChop is an extremely quiet mode of operation for stepper motors, providing absolutely noiseless operation at standstill and low velocities. It's based on voltage mode PWM and is ideal for indoor or home use applications.

### Benefits

- **Silent Operation**: Absolutely noiseless at standstill and low velocities
- **Vibration-Free**: Motor operates free of vibration at low velocities
- **Automatic Tuning**: StealthChop2 automatically adapts to the motor for best performance
- **Energy Efficient**: Can power down motor to very low currents
- **High Dynamics**: Supports high motor dynamics with proper tuning

### When to Use StealthChop

**✅ Recommended for:**
- Indoor or home use applications
- Applications requiring silent operation
- Low-velocity positioning applications
- Applications where noise is a concern
- Applications with varying supply voltage (with automatic tuning)

**❌ Not recommended for:**
- High-velocity applications (use SpreadCycle or combine both)
- Applications requiring StallGuard2 (mutually exclusive)
- Applications with unknown motor characteristics (without proper tuning)
- Applications where current regulation is not satisfying

### Prerequisites

**Critical Requirements:**
1. **Motor Standstill**: Motor must be at standstill when StealthChop is first enabled
2. **Standstill Period**: Keep motor stopped for at least 128 chopper periods after enabling
3. **Current Setting**: IRUN ≥ 8 (current settings below 8 do not work with automatic tuning)
4. **Lower Current Limit**: Motor current must exceed I_LOWER_LIMIT (calculated from TBL, fPWM, VM, R_COIL)
5. **Open Load Check**: Perform open load detection in SpreadCycle before enabling StealthChop

### Automatic Tuning (AT#1 and AT#2)

StealthChop2 features automatic tuning that adapts operating parameters to the motor automatically. Two tuning phases are required:

#### AT#1: Standstill Tuning

**Purpose**: Regulates to nominal current and stores result to PWM_OFS_AUTO.

**Conditions**:
- Motor in standstill
- Actual current scale (CS) identical to run current (IRUN)
- Pin VS at operating level
- Duration: ≤130ms (with internal clock)

**Procedure**:
1. Enable motor with nominal run current (IRUN)
2. Keep motor at standstill for ≥130ms
3. Driver automatically regulates current and stores PWM_OFS_AUTO

**Note**: If standstill reduction is enabled, issue a single step pulse and stop again to power motor to run current.

#### AT#2: Motion Tuning

**Purpose**: Optimizes PWM_GRAD_AUTO during motion.

**Conditions**:
- Move motor at medium velocity (60-300 RPM typical)
- Significant back EMF generated
- Full run current can be reached
- PWM_SCALE_SUM conditions: 1.5*PWM_OFS_AUTO*(IRUN+1)/32 < PWM_SCALE_SUM < 4*PWM_OFS_AUTO*(IRUN+1)/32
- PWM_SCALE_SUM < 255
- Requires 8 fullsteps per change of ±1
- For typical motor with PWM_GRAD_AUTO optimum at 50 or less, up to 400 fullsteps required when starting from default value 0

**Procedure**:
1. Move motor at medium velocity (e.g., during homing procedure)
2. Include constant velocity ramp segment
3. Monitor PWM_SCALE_AUTO going down to zero (indicates successful tuning)
4. Store PWM_GRAD_AUTO to CPU memory for faster tuning procedure (optional)

**Success Indicator**: PWM_SCALE_AUTO approaches 0 during constant velocity phase.

### Configuration Parameters

#### PWM Frequency (`pwm_freq`)

**Purpose**: Chopper frequency selection for StealthChop operation.

**Options**:
- `PWM_FREQ_0`: ~23.4kHz @ 12MHz (lowest frequency)
- `PWM_FREQ_1`: ~35.1kHz @ 12MHz (recommended for most applications)
- `PWM_FREQ_2`: ~46.9kHz @ 12MHz
- `PWM_FREQ_3`: ~58.5kHz @ 12MHz (highest frequency)

**Effect**:
- Lower frequencies: Reduce current ripple, may limit high-velocity performance
- Higher frequencies: Improve high-velocity performance, increase dynamic power dissipation

**Recommendation**: Use lowest setting giving good results. Typical: 1 (~35kHz @ 12MHz).

#### PWM Offset (`pwm_ofs`)

**Purpose**: Initial value for PWM amplitude (offset) for velocity-based scaling.

**Range**: 0-255
- 0: Disables linear current scaling based on current setting
- Typical: 30 (default) for most motors

**Calculation** (for manual mode, `pwm_autoscale=0`):
```
PWM_OFS = 374 * R_COIL * I_COIL / V_M
```

**Note**: With automatic tuning (`pwm_autoscale=1`), this is used as initial value. After AT#1, actual value is stored in PWM_OFS_AUTO.

#### PWM Gradient (`pwm_grad`)

**Purpose**: Velocity-dependent factor to compensate for back EMF.

**Range**: 0-255
- Typical: 0 (default) - let automatic tuning determine

**Calculation** (for manual mode, `pwm_autoscale=0`):
```
PWM_GRAD = C_BEMF * 2π * f_CLK * 1.46 / (V_M * MSPR)
```

**Note**: With automatic tuning (`pwm_autograd=1`), this is used as initial value. After AT#2, actual value is stored in PWM_GRAD_AUTO.

#### Automatic Scaling (`pwm_autoscale`)

**Purpose**: Enable automatic current regulation using current measurement feedback.

- **true (recommended)**: Automatic scaling with current regulator
  - Adapts to motor heating, supply voltage changes
  - Responds to motor stall and load changes
  - Requires current measurement (sense resistors)

- **false**: Feed-forward velocity-controlled mode
  - Very stable amplitude
  - Does not react to supply voltage changes or motor stall
  - Does not require current measurement
  - Only for well-known motor and operating conditions

**Recommendation**: Use automatic mode (true) unless current regulation is not satisfying.

#### Automatic Gradient (`pwm_autograd`)

**Purpose**: Enable automatic tuning of PWM_GRAD_AUTO during AT#2 phase.

- **true (recommended)**: Automatic gradient tuning
  - Driver optimizes PWM_GRAD_AUTO during AT#2
  - Adapts to motor characteristics automatically

- **false**: Use PWM_GRAD from register
  - Use pre-determined PWM_GRAD value
  - Reduces amplitude jitter
  - Requires manual tuning

**Recommendation**: Set to false if you have pre-determined PWM_GRAD and want to reduce jitter.

#### Regulation Coefficient (`pwm_reg`)

**Purpose**: Proportionality coefficient for PWM amplitude regulation loop.

**Range**: 1-15
- Lower values: Stable, soft regulation (slower response)
- Higher values: Faster adaptation (may be less stable)
- Typical: 4 (default) for balanced response
- Minimum: 1 (for slow acceleration during AT#2)

**Recommendation**: Should be as small as possible for stability, but large enough for quick reaction. Optimize for fastest required acceleration/deceleration ramp.

#### PWM Limit (`pwm_lim`)

**Purpose**: Limiting value for current jerk when switching from SpreadCycle to StealthChop.

**Range**: 0-15 (upper 4 bits of 8-bit amplitude limit)
- Lower values: Lower current jerk (smoother switching)
- Higher values: Higher current jerk (faster switching)
- Default: 12 (recommended)

**Recommendation**: Reduce value to yield lower current jerk at mode switching.

### Lower Current Limit

StealthChop current regulator imposes a lower limit for motor current regulation:

```
I_LOWER_LIMIT = t_BLANK * f_PWM * V_M / R_COIL
```

Where:
- `t_BLANK`: Blank time (from TBL setting: 16, 24, 36, or 54 clock cycles)
- `f_PWM`: Chopper frequency (from PWM_FREQ setting)
- `V_M`: Motor supply voltage
- `R_COIL`: Motor coil resistance

**Important**:
- IRUN ≥ 8: Current settings for IRUN below 8 do not work with automatic tuning
- Motor current in AT#1 must exceed I_LOWER_LIMIT
- Lower blank time (TBL) allows lower current limit
- During AT#1, driver may reduce frequency if current cannot be reached

**Example**:
Motor with R_COIL = 5Ω, V_M = 24V, TBL=1 (24 clocks), PWM_FREQ=0 (2/1024 fCLK):
```
I_LOWER_LIMIT = 24 * (2/1024) * 24V / 5Ω = 225mA
```

Motor target current for automatic tuning must be 225mA or more.

### Configuration Examples

#### Basic Configuration (Automatic Tuning)

```cpp
#include "tmc51x0.hpp"

void configureStealthChop() {
    // Configure StealthChop with automatic tuning using intuitive enums
    tmc51x0::StealthChopConfig stealth{};
    
    // Use recommended PWM frequency (~35kHz @ 12MHz)
    stealth.pwm_freq = static_cast<uint8_t>(tmc51x0::StealthChopPwmFreq::PWM_FREQ_1);
    
    // Initial values (will be optimized by automatic tuning)
    stealth.pwm_ofs = 30;  // Typical starting value (normal mode)
    stealth.pwm_grad = 0;  // Let automatic tuning determine
    
    // Enable automatic tuning (recommended)
    stealth.pwm_autoscale = true;
    stealth.pwm_autograd = true;
    
    // Balanced regulation speed (2 increments per half wave)
    stealth.pwm_reg = static_cast<uint8_t>(tmc51x0::StealthChopRegulationSpeed::MODERATE);
    
    // Balanced jerk reduction
    stealth.pwm_lim = static_cast<uint8_t>(tmc51x0::StealthChopJerkReduction::MODERATE);
    
    // Normal freewheeling
    stealth.freewheel = tmc51x0::PWMFreewheel::NORMAL;
    
    // Configure StealthChop
    driver.motorControl.ConfigureStealthChop(stealth);
    
    // IMPORTANT: Motor must be at standstill when StealthChop is first enabled
    // Keep motor stopped for at least 128 chopper periods
    
    // Set velocity threshold for StealthChop (TPWMTHRS)
    // Below this velocity: StealthChop (silent)
    // Above this velocity: SpreadCycle (more torque)
    driver.thresholds.SetModeChangeSpeeds(0.002f, 0.0f, 0.0f, tmc51x0::Unit::RevPerSec);  // Unit::RevPerSec is default
}
```

#### Quick Setup with Helper Constructor (Using Enums)

```cpp
// Most intuitive: Using enums for all parameters
tmc51x0::StealthChopConfig stealth(
    tmc51x0::StealthChopPwmFreq::PWM_FREQ_1,           // PWM frequency
    tmc51x0::StealthChopRegulationSpeed::MODERATE,      // Regulation speed
    tmc51x0::StealthChopJerkReduction::MODERATE        // Jerk reduction
);
driver.motorControl.ConfigureStealthChop(stealth);

// Or with custom initial values (using raw values)
tmc51x0::StealthChopConfig stealth(1, 30, 0, 4, 12);  // freq, ofs, grad, reg, lim
driver.motorControl.ConfigureStealthChop(stealth);
```

#### Advanced Configuration (Fine-Tuned)

```cpp
// For applications requiring fast regulation response
tmc51x0::StealthChopConfig stealth{};
stealth.pwm_freq = static_cast<uint8_t>(tmc51x0::StealthChopPwmFreq::PWM_FREQ_1);
stealth.pwm_reg = static_cast<uint8_t>(tmc51x0::StealthChopRegulationSpeed::FAST);  // Fast response
stealth.pwm_lim = static_cast<uint8_t>(tmc51x0::StealthChopJerkReduction::LOW);     // Allow faster switching
stealth.pwm_autoscale = true;
stealth.pwm_autograd = true;
driver.motorControl.ConfigureStealthChop(stealth);

// For applications requiring very smooth switching
stealth.pwm_reg = static_cast<uint8_t>(tmc51x0::StealthChopRegulationSpeed::SLOW);  // Very stable
stealth.pwm_lim = static_cast<uint8_t>(tmc51x0::StealthChopJerkReduction::MAXIMUM); // Smoothest switching
driver.motorControl.ConfigureStealthChop(stealth);
```

#### Manual Mode (Advanced)

```cpp
// For well-known motor and operating conditions only
tmc51x0::StealthChopConfig stealth{};
stealth.pwm_freq = 1;

// Calculate PWM_OFS: 374 * R_COIL * I_COIL / V_M
stealth.pwm_ofs = static_cast<uint8_t>((374.0f * 5.0f * 1.68f) / 24.0f);  // Example

// Calculate PWM_GRAD from back EMF constant
// PWM_GRAD = C_BEMF * 2π * f_CLK * 1.46 / (V_M * MSPR)
stealth.pwm_grad = /* calculated value */;

// Disable automatic tuning (manual mode)
stealth.pwm_autoscale = false;
stealth.pwm_autograd = false;

driver.motorControl.ConfigureStealthChop(stealth);
```

### Automatic Tuning Procedure

#### Step 1: Initial Setup

```cpp
// Configure StealthChop with automatic tuning enabled
tmc51x0::StealthChopConfig stealth{};
stealth.pwm_freq = 1;
stealth.pwm_autoscale = true;
stealth.pwm_autograd = true;
driver.motorControl.ConfigureStealthChop(stealth);

// Enable StealthChop mode
driver.motorControl.SetStealthChopEnabled(true);

// IMPORTANT: Motor must be at standstill
// Keep motor stopped for at least 128 chopper periods
driver.GetComm().DelayMs(150);  // Ensure AT#1 completes (≤130ms)
```

#### Step 2: AT#1 - Standstill Tuning

```cpp
// Motor should be at standstill with nominal run current (IRUN)
// AT#1 automatically completes during standstill period
// Read automatic tuning results
auto pwm_auto_result = driver.status.GetPwmAuto();
if (pwm_auto_result) {
    uint8_t pwm_ofs_auto = pwm_auto_result.Value();
    ESP_LOGI(TAG, "AT#1 complete: PWM_OFS_AUTO = %u", pwm_ofs_auto);
}
```

#### Step 3: AT#2 - Motion Tuning

```cpp
// Move motor at medium velocity (60-300 RPM typical)
// Include constant velocity ramp segment
auto speed_result = driver.rampControl.SetMaxSpeed(0.02f);  // Unit::RevPerSec is default
if (!speed_result) {
    printf("Error setting max speed: %s\n", speed_result.ErrorMessage());
    return;
}
auto accel_result = driver.rampControl.SetAccelerations(0.01f, 0.01f);  // Unit::RevPerSec is default
if (!accel_result) {
    printf("Error setting accelerations: %s\n", accel_result.ErrorMessage());
    return;
}
auto pos_result = driver.rampControl.SetTargetPosition(10000);
if (!pos_result) {
    printf("Error setting target position: %s\n", pos_result.ErrorMessage());
    return;
}

// Monitor PWM_SCALE_AUTO during motion
while (true) {
    auto reached = driver.rampControl.IsTargetReached();
    if (reached && reached.Value()) {
        break; // Target reached
    }
    if (!reached) {
        ESP_LOGE(TAG, "Error checking target: %s", reached.ErrorMessage());
        break;
    }
    
    auto pwm_result = driver.status.GetPwmScale();
    if (pwm_result) {
        uint8_t pwm_scale_sum = pwm_result.Value();
        // Note: GetPwmScale returns pwm_scale_sum, pwm_scale_auto is available via diagnostics
        // PWM_SCALE_AUTO should approach 0 during constant velocity phase
        ESP_LOGI(TAG, "PWM_SCALE_SUM = %d", pwm_scale_sum);
    }
    driver.GetComm().DelayMs(10);
}

// Read final automatic tuning results
auto pwm_auto_result2 = driver.status.GetPwmAuto();
if (pwm_auto_result2) {
    uint8_t pwm_ofs_auto = pwm_auto_result2.Value();
    ESP_LOGI(TAG, "AT#2 complete: PWM_OFS_AUTO = %u", pwm_ofs_auto);
    // Store values for faster tuning in future (optional)
}
```

### Combining StealthChop and SpreadCycle

For applications requiring high velocity motion, combine StealthChop (low speed) with SpreadCycle (high speed):

```cpp
// Configure StealthChop for low-speed silent operation
tmc51x0::StealthChopConfig stealth{};
stealth.pwm_freq = 1;
driver.motorControl.ConfigureStealthChop(stealth);

// Set velocity threshold (TPWMTHRS)
// Below threshold: StealthChop (silent)
// Above threshold: SpreadCycle (more torque)
driver.thresholds.SetModeChangeSpeeds(200.0f, 0.0f, 0.0f, tmc51x0::Unit::RPM);

// Use low transfer velocity to avoid jerk at switching point
// Typical: 1 to a few 10 RPM
```

**Important**: 
- Set TPWMTHRS to low velocity (1-10 RPM) to avoid jerk at switching point
- Jerk occurs because back-EMF causes phase shift between voltage and current
- At low velocities, jerk can be completely neglected for most motors

### Best Practices

1. **Use Automatic Tuning**: Enable `pwm_autoscale` and `pwm_autograd` for best results
2. **Start at Standstill**: Always enable StealthChop with motor at standstill
3. **Wait for AT#1**: Keep motor stopped for ≥130ms after enabling
4. **Monitor AT#2**: Monitor PWM_SCALE_AUTO during motion tuning
5. **Check Lower Limit**: Ensure motor current exceeds I_LOWER_LIMIT
6. **Open Load Check**: Perform open load detection in SpreadCycle first
7. **Tune Overcurrent**: Tune low-side overcurrent detection for motor stall protection
8. **Store Values**: Store PWM_OFS_AUTO and PWM_GRAD_AUTO for faster future tuning

### Troubleshooting

**Problem**: Motor current too high during deceleration
- **Solution**: Follow automatic tuning process, check optimum tuning conditions

**Problem**: Current regulation not satisfying
- **Solution**: Check that IRUN ≥ 8 and current exceeds I_LOWER_LIMIT

**Problem**: Motor loses steps
- **Solution**: Ensure proper AT#1 and AT#2 completion, check PWM_SCALE_AUTO

**Problem**: Jerk at mode switching
- **Solution**: Use lower TPWMTHRS value (1-10 RPM), reduce PWM_LIM

**Problem**: Open load flags active
- **Solution**: Perform open load test in SpreadCycle before enabling StealthChop

**Problem**: Motor stall causes overcurrent
- **Solution**: Tune low-side overcurrent detection to safely trigger upon motor stall

### Technical Details

**Register Mapping**:
- `PWM_OFS` (bits 7..0): User defined amplitude offset (0-255)
- `PWM_GRAD` (bits 15..8): User defined amplitude gradient (0-255)
- `pwm_freq` (bits 17..16): PWM frequency selection (0-3)
- `pwm_autoscale` (bit 18): Automatic amplitude scaling enable
- `pwm_autograd` (bit 19): Automatic gradient adaptation enable
- `PWM_REG` (bits 27..24): Regulation loop coefficient (1-15)
- `PWM_LIM` (bits 31..28): Amplitude limit for mode switching (0-15)
- `freewheel` (bits 21..20): Freewheeling mode (0-3)

**Automatic Tuning Results** (read-only):
- `PWM_OFS_AUTO`: Automatically determined offset value (0-255)
- `PWM_GRAD_AUTO`: Automatically determined gradient value (0-255)
- `PWM_SCALE_AUTO`: Current regulator correction (-255 to +255, should approach 0)

**Note**: StealthChop requires motor to be at standstill when first enabled. StealthChop and StallGuard2 are mutually exclusive.

## SpreadCycle and Classic Chopper

SpreadCycle is a cycle-by-cycle current control providing superior microstepping quality. It can react extremely fast to changes in motor velocity or motor load. Classic mode is an alternative constant off-time chopper algorithm.

### Benefits

- **Superior Microstepping**: SpreadCycle provides superior microstepping quality even with default settings
- **Fast Response**: Cycle-by-cycle current control reacts extremely fast to velocity/load changes
- **Automatic Optimization**: SpreadCycle automatically determines optimum fast-decay phase length
- **Low Power Dissipation**: Slow decay phases reduce electrical losses and current ripple
- **StallGuard2 Compatible**: Required for StallGuard2 and CoolStep operation

### When to Use SpreadCycle

**✅ Recommended for:**
- Applications requiring StallGuard2 (mutually exclusive with StealthChop)
- Applications requiring CoolStep (requires SpreadCycle)
- High-velocity applications (better than StealthChop at high speeds)
- Applications requiring maximum torque and dynamic performance
- Applications where noise is not a primary concern

**❌ Not recommended for:**
- Applications requiring silent operation (use StealthChop instead)
- Low-velocity positioning where silence is critical

### Chopper Frequency

Chopper frequency is an important parameter. Most motors work optimally in **16-30kHz range**.

**Frequency is influenced by**:
- TOFF setting (slow decay time)
- TBL setting (blank time)
- Hysteresis settings (HSTRT, HEND)
- Motor inductance
- Supply voltage

**Too low frequency**: May generate audible noise
**Too high frequency**: Magnetic losses rise, power dissipation increases

### Configuration Parameters

#### Chopper Mode (`mode`)

**Purpose**: Selects between SpreadCycle and Classic chopper algorithms.

- **SPREAD_CYCLE** (recommended): Patented high-performance algorithm
  - Automatically determines optimum fast-decay phase length
  - Superior microstepping quality with default settings
  - Cycles: on → slow decay → fast decay → slow decay
  
- **CLASSIC**: Constant off-time chopper mode
  - Alternative algorithm, requires more tuning
  - Cycles: on → fast decay → slow decay
  - Used automatically in DcStep fullstepping

#### Off Time (`toff`)

**Purpose**: Sets slow decay time and limits maximum chopper frequency.

**Range**: 0-15
- **0**: Chopper off (driver disabled, motor can free-wheel)
- **1**: Minimum off time (requires TBL ≥ 2)
- **2-15**: Off time setting (typical: 3-5 for 16-30kHz chopper frequency)

**Calculation for target chopper frequency**:
```
t_OFF = (1 / f_chopper) * (slow_decay_percent / 100) * (1 / 2)
TOFF = (t_OFF * f_CLK - 12) / 32
```

**Example** (25kHz target, 50% slow decay, 12MHz clock):
- t_OFF = (1/25000) * 0.5 * 0.5 = 10µs
- TOFF = (10µs * 12MHz - 12) / 32 ≈ 3.0 → use **3**

**Note**: For StealthChop operation, any setting is OK (not used for chopping).

#### Blank Time (`tbl`)

**Purpose**: Masks comparator input to block spikes from parasitic capacitances.

**Options**:
- `TBL_16CLK`: 16 clock cycles (~1.33µs @ 12MHz)
- `TBL_24CLK`: 24 clock cycles (~2.0µs @ 12MHz)
- `TBL_36CLK`: 36 clock cycles (~3.0µs @ 12MHz, typical)
- `TBL_54CLK`: 54 clock cycles (~4.5µs @ 12MHz, for high capacitive loads)

**Recommendation**: Use `TBL_36CLK` for most applications. For highly capacitive loads (filter networks), use `TBL_36CLK` or `TBL_54CLK`.

#### Hysteresis (SpreadCycle Mode)

**Purpose**: Forces minimum current ripple into motor coils for best microstepping results.

**Hysteresis Start (`hstrt`)**: Adds to hysteresis end value (0-7, adds 1-8 to HEND)
- Lower values: Less current ripple (may reduce microstep accuracy)
- Higher values: More current ripple (may increase chopper noise)
- Typical: **4** (effective hysteresis = 4 when HEND=0)

**Hysteresis End (`hend`)**: End value after decrements (0-15 encoded)
- Encoded: 0-2 = negative (-3 to -1), 3 = zero, 4-15 = positive (1 to 12)
- Typical: **0** (effective end = -3)

**Effective Hysteresis** = HSTRT + HEND (with encoding)

**Tuning**:
1. Start from low setting (HSTRT=0, HEND=0)
2. Increase HSTRT until motor runs smoothly at low velocity
3. Check sine wave shape near zero transition (should have no ledge)
4. Too low: Reduced microstep accuracy, humming/vibration at medium velocities
5. Too high: Reduced chopper frequency, increased noise, no benefit

**Note**: Sum HSTRT+HEND must be ≤16 (at CS ≤ 30).

#### Fast Decay Time (Classic Mode Only)

**Purpose**: Sets fixed fast decay time following each on phase.

**Range**: 0-15
- **0**: Slow decay only
- **1-15**: Fast decay time duration
- **Typical**: Similar to slow decay time (TOFF) setting

**Tuning**: Should be long enough to follow falling slope but not cause excess ripple. Tune using oscilloscope or motor smoothness at different velocities.

#### Sine Wave Offset (Classic Mode Only)

**Purpose**: Corrects for zero crossing error in Classic mode.

**Range**: 0-15 (encoded)
- Encoded: 0-2 = negative (-3 to -1), 3 = zero, 4-15 = positive (1 to 12)
- **Typical**: 4-6 (positive offset 1-3) for smoothest operation

**Tuning**: 
- Too low: Motor stands still for short moment during zero crossing
- Too high: Makes larger microstep
- Typically requires positive offset for smoothest operation

#### Passive Fast Decay Time (`tpfd`)

**Purpose**: Adds passive fast decay time after bridge polarity change.

**Range**: 0-15
- **0**: Disabled
- **1-15**: Fast decay time in multiples of 128 clocks (~10µs per unit @ 12MHz)

**Usage**: Starting from 0, increase value if motor suffers from mid-range resonances.

#### Microstep Resolution (`mres`)

**Purpose**: Defines number of microsteps per full step.

**Options**:
- `MRES_256`: 256 microsteps (highest resolution, smoothest)
- `MRES_128`: 128 microsteps
- `MRES_64`: 64 microsteps
- `MRES_32`: 32 microsteps
- `MRES_256`: 256 microsteps (typical, balanced)
- `MRES_8`: 8 microsteps
- `MRES_4`: 4 microsteps
- `MRES_2`: 2 microsteps
- `FULLSTEP`: Full step (no microstepping)

**Note**: Interpolation can be enabled to extrapolate to 256 microsteps regardless of MRES setting.

### Configuration Examples

#### Basic SpreadCycle Configuration

```cpp
#include "tmc51x0.hpp"

void configureSpreadCycle() {
    // Configure SpreadCycle with typical settings
    tmc51x0::ChopperConfig chopper{};
    
    // SpreadCycle mode (recommended)
    chopper.mode = tmc51x0::ChopperMode::SPREAD_CYCLE;
    
    // Off time for ~25kHz chopper frequency (typical)
    chopper.toff = 5;
    
    // Blank time (36 clocks, typical)
    chopper.tbl = static_cast<uint8_t>(tmc51x0::ChopperBlankTime::TBL_36CLK);
    
    // Hysteresis (effective = 4)
    chopper.hstrt = 4;  // Typical
    chopper.hend = 0;   // Typical (effective end = -3)
    
    // Passive fast decay (disabled)
    chopper.tpfd = 0;
    
    // Microstep resolution (256 microsteps)
    chopper.mres = tmc51x0::MicrostepResolution::MRES_256;
    
    // Enable interpolation (recommended)
    chopper.intpol = true;
    
    // Configure chopper
    driver.motorControl.ConfigureChopper(chopper);
}
```

#### Quick Setup with Helper Constructor

```cpp
// SpreadCycle mode with typical settings
tmc51x0::ChopperConfig chopper(5);  // toff=5, other defaults
driver.motorControl.ConfigureChopper(chopper);

// Or with custom values
tmc51x0::ChopperConfig chopper(5, 2, 4, 0, tmc51x0::MicrostepResolution::MRES_256);  // toff, tbl, hstrt, hend, mres
driver.motorControl.ConfigureChopper(chopper);
```

#### Classic Mode Configuration

```cpp
// Classic constant off-time chopper
tmc51x0::ChopperConfig chopper{};
chopper.mode = tmc51x0::ChopperMode::CLASSIC;
chopper.toff = 5;              // Slow decay time
chopper.tfd = 5;              // Fast decay time (similar to toff)
chopper.hend = 4;             // Sine wave offset (positive offset for zero crossing)
chopper.disfdcc = false;      // Enable comparator termination
chopper.tbl = static_cast<uint8_t>(tmc51x0::ChopperBlankTime::TBL_36CLK);
chopper.mres = static_cast<uint8_t>(tmc51x0::MicrostepResolution::MRES_256);
chopper.intpol = true;
driver.motorControl.ConfigureChopper(chopper);

// Or use helper constructor
tmc51x0::ChopperConfig chopper(5, 5, 4);  // toff, tfd, offset
driver.motorControl.ConfigureChopper(chopper);
```

#### High-Velocity Configuration

```cpp
// Optimized for high velocities
tmc51x0::ChopperConfig chopper{};
chopper.mode = tmc51x0::ChopperMode::SPREAD_CYCLE;
chopper.toff = 2;              // Shorter off time (higher frequency)
chopper.tbl = static_cast<uint8_t>(tmc51x0::ChopperBlankTime::TBL_24CLK);  // Shorter blank time
chopper.hstrt = 4;
chopper.hend = 0;
chopper.mres = static_cast<uint8_t>(tmc51x0::MicrostepResolution::MRES_256);
chopper.intpol = true;
driver.motorControl.ConfigureChopper(chopper);
```

### Tuning Guide

#### Step 1: Calculate TOFF for Target Frequency

```cpp
// Target: 25kHz chopper frequency, 50% slow decay, 12MHz clock
float target_freq_hz = 25000.0f;
float slow_decay_percent = 50.0f;
float f_clk_hz = 12000000.0f;

float t_off_us = (1.0f / target_freq_hz) * (slow_decay_percent / 100.0f) * 0.5f * 1000000.0f;
uint8_t toff = static_cast<uint8_t>((t_off_us * f_clk_hz / 1000000.0f - 12.0f) / 32.0f);
toff = constrain<uint8_t>(toff, 1U, 15U);
```

#### Step 2: Tune Hysteresis (SpreadCycle)

```cpp
// Start from low setting
chopper.hstrt = 0;
chopper.hend = 0;

// Increase until motor runs smoothly at low velocity
for (uint8_t hstrt = 0; hstrt <= 7; hstrt++) {
    chopper.hstrt = hstrt;
    driver.motorControl.ConfigureChopper(chopper);
    
    // Test motor smoothness at low velocity
    // Move motor and check for smooth operation
    auto speed_result = driver.rampControl.SetMaxSpeed(0.002f);  // Unit::RevPerSec is default
    if (!speed_result) {
        printf("Error setting max speed: %s\n", speed_result.ErrorMessage());
        continue;
    }
    auto pos_result = driver.rampControl.SetTargetPosition(1000);
    if (!pos_result) {
        printf("Error setting target position: %s\n", pos_result.ErrorMessage());
        continue;
    }
    // ... wait for motion and check smoothness ...
    // If smooth, this is the optimal setting
    // break;
}
```

#### Step 3: Tune TPFD (if resonances occur)

```cpp
// If motor suffers from mid-range resonances
for (uint8_t tpfd = 1; tpfd <= 15; tpfd++) {
    chopper.tpfd = tpfd;
    driver.motorControl.ConfigureChopper(chopper);
    
    // Test for resonances at mid-range velocities
    auto speed_result = driver.rampControl.SetMaxSpeed(0.01f);  // Unit::RevPerSec is default
    if (!speed_result) {
        printf("Error setting max speed: %s\n", speed_result.ErrorMessage());
        continue;
    }
    auto pos_result = driver.rampControl.SetTargetPosition(5000);
    if (!pos_result) {
        printf("Error setting target position: %s\n", pos_result.ErrorMessage());
        continue;
    }
    // ... wait for motion and check for resonances ...
    // If resonances reduced, this is the optimal setting
    // break;
}
```

### Best Practices

1. **Use SpreadCycle**: Recommended for most applications (superior microstepping quality)
2. **Target Frequency**: Aim for 16-30kHz chopper frequency
3. **Start Conservative**: Begin with typical settings (TOFF=5, TBL=36CLK, HSTRT=4)
4. **Tune Hysteresis**: Start from low and increase until smooth operation
5. **Enable Interpolation**: Always enable for best microstepping quality
6. **Match TOFF and TBL**: For high velocities, use shorter TOFF (2-3) and TBL (24CLK)

### Troubleshooting

**Problem**: Audible chopper noise
- **Solution**: Increase TOFF (reduces frequency), check that frequency is in 16-30kHz range

**Problem**: Reduced microstep accuracy
- **Solution**: Increase hysteresis (HSTRT), check sine wave shape near zero crossing

**Problem**: Motor humming/vibration at medium velocities
- **Solution**: Increase hysteresis (HSTRT), ensure HSTRT+HEND ≥ 4

**Problem**: Mid-range resonances
- **Solution**: Increase TPFD (passive fast decay time)

**Problem**: Chopper frequency too low
- **Solution**: Reduce TOFF, reduce TBL, reduce hysteresis

**Problem**: Chopper frequency too high
- **Solution**: Increase TOFF, increase TBL, increase hysteresis

### Technical Details

**Register Mapping** (CHOPCONF register):
- `TOFF` (bits 3..0): Off time and driver enable (0-15)
- `HSTRT/TFD[2:0]` (bits 6..4): Hysteresis start (SpreadCycle) or fast decay time (Classic)
- `HEND/OFFSET` (bits 10..7): Hysteresis end (SpreadCycle) or sine wave offset (Classic)
- `TFD[3]` (bit 11): MSB of fast decay time (Classic only)
- `disfdcc` (bit 12): Disable fast decay comparator (Classic only)
- Reserved (bit 13): Reserved, set to 0
- `chm` (bit 14): Chopper mode (0=SpreadCycle, 1=Classic)
- `TBL` (bits 16..15): Comparator blank time (0-3)
- Reserved (bit 17): Reserved, set to 0
- `vhighfs` (bit 18): High velocity fullstep selection
- `vhighchm` (bit 19): High velocity chopper mode
- `TPFD` (bits 23..20): Passive fast decay time (0-15)
- `MRES` (bits 27..24): Microstep resolution (0-8)
- `intpol` (bit 28): Interpolation to 256 microsteps
- `dedge` (bit 29): Enable double edge step pulses
- `diss2g` (bit 30): Short to GND protection disable
- `diss2vs` (bit 31): Short to supply protection disable

**Chopper Cycles**:
- **SpreadCycle**: on → slow decay → fast decay → slow decay (4 phases)
- **Classic**: on → fast decay → slow decay (3 phases)

**Note**: SpreadCycle is required for StallGuard2 and CoolStep operation. StealthChop and SpreadCycle can be combined using velocity thresholds (TPWMTHRS).

## Freewheeling Mode

Freewheeling controls motor behavior when `ihold=0` (no hold current).

### Freewheeling Options

```cpp
void configureFreewheeling() {
    // Configure freewheeling via StealthChopConfig
    tmc51x0::StealthChopConfig stealth{};
    stealth.freewheel = tmc51x0::PWMFreewheel::NORMAL;  // Normal operation (coast when ihold=0)
    driver.motorControl.ConfigureStealthChop(stealth);
    
    // Or change freewheeling mode at runtime
    stealth.freewheel = tmc51x0::PWMFreewheel::ENABLED;  // Freewheeling enabled (low resistance)
    driver.motorControl.ConfigureStealthChop(stealth);
    
    stealth.freewheel = tmc51x0::PWMFreewheel::SHORT_LS;  // Coil shorted using low-side drivers
    driver.motorControl.ConfigureStealthChop(stealth);
    
    stealth.freewheel = tmc51x0::PWMFreewheel::SHORT_HS;  // Coil shorted using high-side drivers
    driver.motorControl.ConfigureStealthChop(stealth);
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
    tmc51x0::ReferenceSwitchConfig ref_switch{};
    
    // Configure switches with enum-based API
    ref_switch.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH;
    ref_switch.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH;
    ref_switch.latch_left = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;
    ref_switch.latch_right = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;
    ref_switch.stop_mode = tmc51x0::ReferenceStopMode::SOFT_STOP;
    ref_switch.latch_left = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;
    ref_switch.latch_right = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;
    
    driver.switches.ConfigureReferenceSwitch(ref_switch);
}
```

### Reading Latched Position

```cpp
void homeToEndstop() {
    // Move toward endstop
    auto mode_result = driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_NEG);
    if (!mode_result) {
        printf("Error setting ramp mode: %s\n", mode_result.ErrorMessage());
        return;
    }
    auto speed_result = driver.rampControl.SetMaxSpeed(500.0f);
    if (!speed_result) {
        printf("Error setting max speed: %s\n", speed_result.ErrorMessage());
        return;
    }
    auto enable_result = driver.motorControl.Enable();
    if (!enable_result) {
        printf("Error enabling motor: %s\n", enable_result.ErrorMessage());
        return;
    }
    
    // Wait for endstop (check RAMP_STAT register)
    // Or use GetLatchedPosition() after switch triggers
    
    auto latched_result = driver.switches.GetLatchedPosition(tmc51x0::Unit::Steps);
    if (latched_result) {
        float latched_pos_steps = latched_result.Value();
        (void)latched_pos_steps;
        auto set_pos_result = driver.rampControl.SetCurrentPosition(0.0f, tmc51x0::Unit::Steps);  // Set as home
        if (!set_pos_result) {
            printf("Error setting current position: %s\n", set_pos_result.ErrorMessage());
            return;
        }
    } else {
        printf("Error reading latched position: %s\n", latched_result.ErrorMessage());
        return;
    }
}
```

## Microstep Lookup Tables

Custom microstep lookup tables allow fine-tuning of microstep interpolation for smoother motion.

### Default Lookup Table

The TMC51x0 (TMC5130 & TMC5160) uses a default sinusoidal lookup table. For most applications, this is sufficient.

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
    driver.events.SetXCompare(1000.0f, tmc51x0::Unit::Steps);
    
    // The pulse appears on SWP_DIAG1 output pin
    // Useful for triggering external events at specific positions
}
```

## Lost Steps Counter

When dcStep is enabled, the driver can count lost steps.

### Reading Lost Steps

```cpp
void checkStepLoss() {
    uint32_t lost_steps = driver.status.GetLostSteps();
    
    if (lost_steps > 0) {
        printf("Warning: %u steps lost\n", lost_steps);
        // Handle step loss (re-home, adjust current, etc.)
    }
}
```

**Note**: Lost steps counter only works when `SD_MODE=1` (external step/dir mode).

## Complete Advanced Setup Example

```cpp
#include "tmc51x0.hpp"

void setupAdvancedMotor() {
    // Basic initialization
    tmc51x0::DriverConfig cfg{};
    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Initialization error: %s\n", init_result.ErrorMessage());
        return; // or handle error appropriately
    }
    
    // Configure CoolStep
    tmc51x0::CoolStepConfig coolstep{};
    coolstep.lower_threshold_sg = 64;   // Automatically converts to SEMIN=2
    coolstep.upper_threshold_sg = 256;  // Automatically converts to SEMAX=5
    coolstep.increment_step = tmc51x0::CoolStepIncrementStep::STEP_2;
    coolstep.decrement_speed = tmc51x0::CoolStepDecrementSpeed::EVERY_2;
    auto coolstep_result = driver.motorControl.ConfigureCoolStep(coolstep);
    if (!coolstep_result) {
        printf("Error configuring CoolStep: %s\n", coolstep_result.ErrorMessage());
        return;
    }
    
    // Configure reference switches
    tmc51x0::ReferenceSwitchConfig ref_switch{};
    ref_switch.left_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
    ref_switch.right_switch_active = tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;
    ref_switch.stop_mode = tmc51x0::ReferenceStopMode::SOFT_STOP;
    ref_switch.latch_left = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;
    ref_switch.latch_right = tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;
    auto ref_result = driver.switches.ConfigureReferenceSwitch(ref_switch);
    if (!ref_result) {
        printf("Error configuring reference switch: %s\n", ref_result.ErrorMessage());
        return;
    }
    
    // Set mode change speeds (StealthChop, CoolStep, High-speed thresholds + unit)
    auto speed_result = driver.thresholds.SetModeChangeSpeeds(
        200.0f,   // stealthChop threshold (TPWMTHRS)
        30.0f,    // CoolStep threshold (TCOOLTHRS)
        300.0f,   // High-speed threshold (THIGH)
        tmc51x0::Unit::RPM
    );
    if (!speed_result) {
        printf("Error setting mode change speeds: %s\n", speed_result.ErrorMessage());
        return;
    }
    
    // Enable motor
    auto enable_result = driver.motorControl.Enable();
    if (!enable_result) {
        printf("Error enabling motor: %s\n", enable_result.ErrorMessage());
        return;
    }
}
```

## Troubleshooting

### CoolStep Not Activating

**Problem**: Current doesn't reduce despite low load

**Solution**:
- Verify `TCOOLTHRS` is set correctly
- Check that motor speed is between `TCOOLTHRS` and `THIGH`
- Ensure StallGuard2 is properly configured
- Verify StallGuard2 threshold is appropriate

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
- Check `left_switch_active` / `right_switch_active` are set (not DISABLED)
- Verify switch signal reaches driver (check IO_INPUT register)
- Ensure `stop_mode` is SOFT_STOP for smooth stopping

## See Also

- [Examples](examples.md) -- Progressively harder walkthroughs including StealthChop, StallGuard, and fatigue testing
- [Sensorless Homing](special_features_sensorless_homing.md) -- StallGuard2 homing and bounds finding
- [API Reference](api_reference.md) -- Complete method signatures for all subsystems
- [ESP32 Example Source Code](../examples/esp32/main/) -- Production implementations you can build and flash

---

**Navigation**
[<- Sensorless Homing](special_features_sensorless_homing.md) | [Next: Troubleshooting ->](troubleshooting.md) | [Back to Index](index.md)

