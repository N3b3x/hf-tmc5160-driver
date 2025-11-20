---
layout: default
title: "⚙️ Configuration"
description: "Configuration options for the TMC5160 driver"
nav_order: 5
parent: "📚 Documentation"
permalink: /docs/configuration/
---

# Configuration

This guide covers all configuration options available for the TMC5160 driver.

## Compile-Time Configuration

### Debug Logging

You can disable debug logging at compile time to reduce code size:

```cpp
#define TMC5160_DISABLE_DEBUG_LOGGING
#include "inc/tmc5160.hpp"
```

When disabled, all debug logging code is optimized out completely.

## Runtime Configuration

### Driver Configuration Structure

The driver uses a `DriverConfig` structure for initialization:

```cpp
tmc5160::DriverConfig cfg{};

// Power stage configuration
cfg.power_stage.drv_strength = 2;  // Gate driver current (0-3)
cfg.power_stage.bbm_time = 0;      // Break-before-make time (0-24 ns)
cfg.power_stage.bbm_clks = 4;      // BBM clock cycles (0-15)

// Motor current configuration
cfg.motor.global_scaler = 32;      // Global current scaler (32-256)
cfg.motor.irun = 20;               // Run current (0-31, recommended 16-31)
cfg.motor.ihold = 10;              // Hold current (0-31, typically 70% of irun)

// Chopper configuration
cfg.chopper.toff = 5;              // Off time (0-15, 0=disabled)
cfg.chopper.hstrt = 4;             // Hysteresis start (0-7)
cfg.chopper.hend = 0;              // Hysteresis end (0-15)
cfg.chopper.tbl = 2;               // Blank time (0-3)
cfg.chopper.vsense = true;         // High sensitivity mode
cfg.chopper.mres = 4;              // Microstep resolution (0-8, 4=16 microsteps)
cfg.chopper.intpol = true;         // Enable interpolation

// StealthChop configuration
cfg.stealthchop.pwm_ofs = 30;      // PWM offset (0-255)
cfg.stealthchop.pwm_grad = 0;      // PWM gradient (0-255)
cfg.stealthchop.pwm_freq = 1;      // PWM frequency (0-3)
cfg.stealthchop.pwm_autoscale = true;  // Enable auto-scaling
cfg.stealthchop.pwm_autograd = true;   // Enable auto-gradient

// Short protection
cfg.short_protection.s2vs_level = 6;   // Short to VS sensitivity (4-15)
cfg.short_protection.s2g_level = 6;    // Short to GND sensitivity (2-15)
cfg.short_protection.shortfilter = 1; // Filter bandwidth (0-3)
cfg.short_protection.shortdelay = 0;   // Detection delay (0-1)

// Motor direction
cfg.direction = tmc5160::MotorDirection::NORMAL;

// Global configuration (GCONF register)
cfg.global_config.en_pwm_mode = true;  // Enable stealthChop
cfg.global_config.multistep_filt = true; // Enable step filtering
cfg.global_config.shaft = false;  // Normal direction

// Ramp parameters
cfg.ramp_params.tpowerdown = 10;  // Power down delay
cfg.ramp_params.tzerowait = 0;    // Zero wait time
cfg.ramp_params.a1 = 0.0f;         // Use AMAX for first acceleration

// Clock frequency
cfg.f_clk = 12000000;  // 12 MHz (default)

driver.Initialize(cfg);
```

## Configuration Methods

### Ramp Control Configuration

```cpp
// Set ramp mode
driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);

// Set motion parameters
driver.rampControl.SetTargetPosition(1000);
driver.rampControl.SetMaxSpeed(1000.0f);      // steps/s
driver.rampControl.SetAcceleration(500.0f);   // steps/s²
driver.rampControl.SetRampSpeeds(0.0f, 0.1f, 0.0f); // start, stop, transition
```

### Motor Control Configuration

```cpp
// Set motor currents
driver.motorControl.SetCurrent(20, 10);  // irun, ihold

// Configure chopper
tmc5160::ChopperConfig chop_cfg{};
chop_cfg.toff = 5;
chop_cfg.mres = 4;  // 16 microsteps
driver.motorControl.ConfigureChopper(chop_cfg);

// Configure stealthChop
tmc5160::StealthChopConfig stealth_cfg{};
stealth_cfg.pwm_autoscale = true;
driver.motorControl.ConfigureStealthChop(stealth_cfg);

// Set mode change speeds
driver.motorControl.SetModeChangeSpeeds(100.0f, 500.0f, 2000.0f);
// pwm_thrs: stealthChop threshold
// cool_thrs: coolStep threshold
// high_thrs: high-speed mode threshold
```

### Encoder Configuration

```cpp
tmc5160::EncoderConfig enc_cfg{};
enc_cfg.pol_n = true;           // N channel active high
enc_cfg.ignore_ab = true;       // Ignore A/B polarity
enc_cfg.sensitivity = 0;         // No edge sensitivity
enc_cfg.enc_sel_decimal = false; // Binary mode
driver.encoder.Configure(enc_cfg);

// Set encoder resolution
driver.encoder.SetResolution(200, 1000, false); // 200 steps/rev, 1000 pulses/rev

// Set allowed deviation
driver.encoder.SetAllowedDeviation(10); // 10 steps
```

### StallGuard Configuration

```cpp
tmc5160::StallGuardConfig sg_cfg{};
sg_cfg.sgt = 0;        // StallGuard threshold (-64 to 63)
sg_cfg.semin = 0;      // Minimum SG value (0-15)
sg_cfg.semax = 0;      // SG hysteresis (0-15)
sg_cfg.sfilt = false;  // Enable filter
driver.diagnostics.ConfigureStallGuard(sg_cfg);
```

## Default Values

| Option | Default | Description |
|--------|---------|-------------|
| `power_stage.drv_strength` | 2 | Gate driver current |
| `power_stage.bbm_time` | 0 | BBM time in ns |
| `power_stage.bbm_clks` | 4 | BBM clock cycles |
| `motor.global_scaler` | 32 | Global current scaler |
| `motor.irun` | 16 | Run current |
| `motor.ihold` | 0 | Hold current |
| `chopper.toff` | 5 | Off time |
| `chopper.mres` | 4 | 16 microsteps |
| `stealthchop.pwm_autoscale` | true | Auto-scaling enabled |
| `short_protection.s2vs_level` | 6 | Short to VS sensitivity |
| `short_protection.s2g_level` | 6 | Short to GND sensitivity |
| `f_clk` | 12000000 | Clock frequency (12 MHz) |

## Recommended Settings

### For Silent Operation (stealthChop)

```cpp
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;
cfg.motor.ihold = 10;
cfg.chopper.toff = 5;
cfg.chopper.mres = 4;  // 16 microsteps
cfg.stealthchop.pwm_autoscale = true;
cfg.stealthchop.pwm_autograd = true;
driver.Initialize(cfg);

// Enable stealthChop
driver.motorControl.SetModeChangeSpeeds(100.0f, 0.0f, 0.0f);
```

### For High Torque (spreadCycle)

```cpp
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 25;
cfg.motor.ihold = 15;
cfg.chopper.toff = 5;
cfg.chopper.mres = 3;  // 32 microsteps
cfg.chopper.chm = false; // spreadCycle mode
driver.Initialize(cfg);

// Disable stealthChop (use spreadCycle)
driver.motorControl.SetModeChangeSpeeds(0.0f, 0.0f, 0.0f);
```

### For Closed-Loop Control (with Encoder)

```cpp
tmc5160::DriverConfig cfg{};
cfg.motor.irun = 20;
cfg.motor.ihold = 10;
driver.Initialize(cfg);

// Configure encoder
tmc5160::EncoderConfig enc_cfg{};
enc_cfg.enc_sel_decimal = false; // Binary mode
driver.encoder.Configure(enc_cfg);
driver.encoder.SetResolution(200, 1000, false);
driver.encoder.SetAllowedDeviation(10);
```

## Next Steps

- See [Examples](examples.md) for configuration examples
- Review [API Reference](api_reference.md) for all configuration methods

---

**Navigation**
⬅️ [Platform Integration](platform_integration.md) | [Next: API Reference ➡️](api_reference.md) | [Back to Index](index.md)

