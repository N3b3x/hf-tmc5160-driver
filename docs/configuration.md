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

// Power stage configuration (user-friendly physical parameters)
cfg.power_stage.mosfet_miller_charge_nc = 30.0f;  // MOSFET Miller charge in nC (0 = auto-calculate, auto-calculates DRVSTRENGTH)
cfg.power_stage.bbm_time_ns = 200;                 // Break-before-make time in nanoseconds (0 = auto-calculate, auto-calculates BBMTIME/BBMCLKS)
cfg.power_stage.sense_filter = tmc5160::SenseFilterTime::T100ns; // Sense amplifier filter time constant
cfg.power_stage.over_temp_protection = tmc5160::OverTempProtection::Level0; // Over-temperature protection (150°C threshold)

// Short protection (user-friendly voltage thresholds and timing)
cfg.power_stage.s2vs_voltage_mv = 625;  // Short to VS voltage threshold in mV (0 = auto = 625mV, equivalent to S2VS_LEVEL=6)
cfg.power_stage.s2g_voltage_mv = 625;  // Short to GND voltage threshold in mV (0 = auto = 625mV, equivalent to S2G_LEVEL=6)
cfg.power_stage.shortfilter = 1;        // Spike filter bandwidth (0=100ns, 1=1µs, 2=2µs, 3=3µs)
cfg.power_stage.short_detection_delay_us_x10 = 0;  // Detection delay in 0.1µs units (0 = auto = 0.85µs)

// Motor specification (user-friendly physical parameters)
cfg.motor_spec.steps_per_rev = 200;
cfg.motor_spec.rated_current_ma = 1680;
cfg.motor_spec.sense_resistor_mohm = 50;  // 0.05Ω
cfg.motor_spec.supply_voltage_mv = 24000;  // 24V

// Optional: Percentage adjustments for calculated current settings
cfg.motor_spec.scaler_adjustment_percent = 0.0f;  // Adjust GLOBAL_SCALER by percentage
cfg.motor_spec.irun_adjustment_percent = 0.0f;    // Adjust IRUN by percentage
cfg.motor_spec.ihold_adjustment_percent = 0.0f;   // Adjust IHOLD by percentage
// Note: IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated during initialization

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
| `power_stage.mosfet_miller_charge_nc` | 10.0f | MOSFET Miller charge in nC (0 = auto-calculate, auto-calculates DRVSTRENGTH) |
| `power_stage.bbm_time_ns` | 100 | Break-before-make time in nanoseconds (0 = auto-calculate, auto-calculates BBMTIME/BBMCLKS) |
| `power_stage.sense_filter` | T100ns | Sense amplifier filter time constant enum |
| `power_stage.over_temp_protection` | Level0 | Over-temperature protection level (150°C threshold) |
| `power_stage.s2vs_voltage_mv` | 0 (auto=625) | Short to VS voltage threshold in mV (0 = auto = 625mV, equivalent to S2VS_LEVEL=6) |
| `power_stage.s2g_voltage_mv` | 0 (auto=625) | Short to GND voltage threshold in mV (0 = auto = 625mV, equivalent to S2G_LEVEL=6) |
| `power_stage.shortfilter` | 1 | Spike filter bandwidth (0=100ns, 1=1µs, 2=2µs, 3=3µs) |
| `power_stage.short_detection_delay_us_x10` | 0 (auto=8.5) | Detection delay in 0.1µs units (0 = auto = 0.85µs) |
| `motor_spec.steps_per_rev` | 200 | Steps per revolution |
| `motor_spec.rated_current_ma` | 1500 | Rated motor current in mA |
| `motor_spec.sense_resistor_mohm` | 50 | Sense resistor in mΩ (0.05Ω) |
| `motor_spec.supply_voltage_mv` | 24000 | Supply voltage in mV (24V) |
| `motor_spec.scaler_adjustment_percent` | 0.0 | Percentage adjustment for GLOBAL_SCALER (-50.0 to +50.0) |
| `motor_spec.irun_adjustment_percent` | 0.0 | Percentage adjustment for IRUN (-50.0 to +50.0) |
| `motor_spec.ihold_adjustment_percent` | 0.0 | Percentage adjustment for IHOLD (-50.0 to +50.0) |
| **Note** | | IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated during initialization |
| `chopper.toff` | 5 | Off time |
| `chopper.mres` | 4 | 16 microsteps |
| `stealthchop.pwm_autoscale` | true | Auto-scaling enabled |
| `f_clk` | 12000000 | Clock frequency (12 MHz) |

## Power Stage Parameter Conversion

The driver automatically converts user-friendly physical parameters to register values based on datasheet specifications:

### Short Protection Voltage Thresholds

The driver converts voltage thresholds (mV) to register levels using interpolation based on datasheet typical values:

**S2VS (Short to VS) Voltage Thresholds:**
- **S2VS_LEVEL=6**: 550-625-700mV (recommended, default)
- **S2VS_LEVEL=15**: 1400-1560-1720mV (lowest sensitivity)
- Linear interpolation between levels 4-15

**S2G (Short to GND) Voltage Thresholds:**
- **S2G_LEVEL=6 (VS<50V)**: 460-625-800mV (recommended, default)
- **S2G_LEVEL=15 (VS<52V)**: 1200-1560-1900mV
- **S2G_LEVEL=15 (VS<55V)**: 850mV (minimum for VS>52V to prevent false triggers)
- **Important**: For VS>52V, minimum recommended is 1200mV (S2G_LEVEL=12) to prevent false triggers

**Detection Delay Timing:**
- **shortdelay=0**: 0.5-0.85-1.1µs (normal, recommended, default)
- **shortdelay=1**: 1.1-1.6-2.2µs (high delay)
- Threshold at ~1.0µs: below uses shortdelay=0, at or above uses shortdelay=1

### BBM Time Conversion

Break-before-make time is converted from nanoseconds to BBMTIME/BBMCLKS register values:

- **BBMTIME=0**: 75-100ns (shortest, typical 100ns)
- **BBMTIME=16**: 200ns
- **BBMTIME=24**: 375-500ns (longest, typical 375ns)
- **BBMCLKS**: Used for times >200ns (digital delay in clock cycles)

The driver adds 30% headroom as recommended by the datasheet to cover production variations.

### MOSFET Miller Charge to DRVSTRENGTH

The driver selects DRVSTRENGTH based on MOSFET Miller charge:
- **DRVSTRENGTH=0**: Weak (default, for small MOSFETs <10nC)
- **DRVSTRENGTH=1**: Weak+TC (medium above OTPW)
- **DRVSTRENGTH=2**: Medium (for medium MOSFETs ~30nC)
- **DRVSTRENGTH=3**: Strong (for large MOSFETs >50nC)

See [`inc/tmc5160_motor_calc.hpp`](../inc/tmc5160_motor_calc.hpp) for conversion function implementations.

## Recommended Settings

### For Silent Operation (stealthChop)

```cpp
tmc5160::DriverConfig cfg{};
cfg.motor_spec.rated_current_ma = 1680;
cfg.motor_spec.sense_resistor_mohm = 50;
cfg.motor_spec.supply_voltage_mv = 24000;
cfg.motor_spec.global_scaler = 0;  // Auto-calculate
cfg.motor_spec.irun = 0;           // Auto-calculate
cfg.motor_spec.ihold = 0;          // Auto-calculate
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
cfg.motor_spec.user.rated_current_ma = 2000;
cfg.motor_spec.user.sense_resistor_mohm = 50;
cfg.motor_spec.user.supply_voltage_mv = 24000;
cfg.motor_spec.global_scaler = 0;  // Auto-calculate
cfg.motor_spec.irun = 0;           // Auto-calculate
cfg.motor_spec.ihold = 0;          // Auto-calculate
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
cfg.motor_spec.rated_current_ma = 1680;
cfg.motor_spec.sense_resistor_mohm = 50;
cfg.motor_spec.supply_voltage_mv = 24000;
cfg.motor_spec.global_scaler = 0;  // Auto-calculate
cfg.motor_spec.irun = 0;           // Auto-calculate
cfg.motor_spec.ihold = 0;          // Auto-calculate
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

