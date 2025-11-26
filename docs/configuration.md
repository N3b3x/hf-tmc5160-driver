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

### ESP32 Configuration Hierarchy (Recommended for Examples)

For ESP32 examples, configurations are organized into three layers defined at compile time in `esp32_tmc5160_test_config.hpp`:

**1. Board Configuration** (`BoardConfig_*` namespaces):
- **Board-specific**: Hardware parameters that stay the same for the same driver board
- **Available Boards**:
  - `BOARD_TMC5160_EVAL`: TMC5160 Evaluation Kit (0.05Ω sense, BSC072N08NS5 MOSFETs)
  - `BOARD_TMC5160_BOB`: TMC5160 Break-Out Board (0.11Ω sense, typical MOSFETs)
- **Board Parameters**:
  - `SENSE_RESISTOR_MOHM`: Sense resistor value (50 mOhm for EVAL, 110 mOhm for BOB)
  - `SUPPLY_VOLTAGE_MV`: Motor supply voltage (24000 mV = 24V)
  - `CLOCK_FREQUENCY_HZ`: TMC5160 clock frequency (12000000 Hz = 12 MHz)
  - `MOSFET_MILLER_CHARGE_NC`: MOSFET Miller charge (6.0 nC for EVAL, 30.0 nC for BOB)
  - `BBM_TIME_NS`: Break-before-make time (100 ns for EVAL, 200 ns for BOB)
  - Short protection defaults

**2. Motor Configurations** (per-motor namespaces):
- **Motor-specific**: Parameters that depend on the motor being used
- `MotorConfig_17HS4401S`: 17HS4401S with 5.18:1 gearbox
- `MotorConfig_17HS4401S_Direct`: 17HS4401S direct drive
- `MotorConfig_AppliedMotion_5034_369`: Applied Motion 5034-369 NEMA 34

**3. Platform Configuration** (`PlatformConfig` namespace):
- **Platform-specific**: Parameters that depend on the application/platform
- Reference switches (endstops) configuration
- Encoder configuration
- Mechanical system type (DirectDrive, LeadScrew, BeltDrive, Gearbox)
- Lead screw pitch, belt parameters (if applicable)

**Helper Functions**:
- `ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg)`: Configure motor + board
- `ConfigureDriverFromMotor_17HS4401S_Direct(cfg)`: Configure motor + board
- `ConfigureDriverFromMotor_AppliedMotion_5034(cfg)`: Configure motor + board
- `ApplyPlatformConfig(cfg)`: Apply platform configuration
- `GetReferenceSwitchConfig()`: Get reference switch config from PlatformConfig
- `GetEncoderConfig()`: Get encoder config from PlatformConfig

See [`examples/esp32/docs/driver_configuration_guide.md`](../examples/esp32/docs/driver_configuration_guide.md) for detailed usage.

### Debug Logging

You can disable debug logging at compile time to reduce code size:

```cpp
#define TMC5160_DISABLE_DEBUG_LOGGING
#include "inc/tmc5160.hpp"
```

When disabled, all debug logging code is optimized out completely.

## Runtime Configuration

### Driver Configuration Structure

The driver uses a `DriverConfig` structure for initialization. **Current settings (IRUN, IHOLD, GLOBAL_SCALER) are automatically calculated** from motor physical specifications during initialization.

#### Recommended Approach: Using Helper Functions (ESP32 Examples)

For ESP32 examples, use the helper functions from `esp32_tmc5160_test_config.hpp`:

```cpp
#include "esp32_tmc5160_test_config.hpp"

// Select motor, board, and platform at compile time
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;
static constexpr tmc5160_test_config::BoardType SELECTED_BOARD = 
    tmc5160_test_config::BoardType::BOARD_TMC5160_EVAL;  // or BOARD_TMC5160_BOB
static constexpr tmc5160_test_config::PlatformType SELECTED_PLATFORM = 
    tmc5160_test_config::PlatformType::PLATFORM_TEST_RIG;

// 1. Configure motor
tmc5160::DriverConfig cfg{};

if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
} else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    tmc5160_test_config::ConfigureDriverFromMotor_17HS4401S_Direct(cfg);
} else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    tmc5160_test_config::ConfigureDriverFromMotor_AppliedMotion_5034(cfg);
}

// 2. Apply board configuration (sense resistor, supply voltage, MOSFETs, etc.)
tmc5160_test_config::ApplyBoardConfig<SELECTED_BOARD>(cfg);

// 3. Apply platform configuration (mechanical system type, etc.)
tmc5160_test_config::ApplyPlatformConfig<SELECTED_PLATFORM>(cfg);

// 4. Initialize driver (current settings calculated automatically)
driver.Initialize(cfg);

// 5. Configure platform-specific features after initialization
// Reference switches
auto ref_cfg = tmc5160_test_config::GetReferenceSwitchConfig<SELECTED_PLATFORM>();
driver.rampControl.ConfigureReferenceSwitch(ref_cfg);

// Encoder
auto enc_cfg = tmc5160_test_config::GetEncoderConfig<SELECTED_PLATFORM>();
driver.encoder.Configure(enc_cfg);
driver.encoder.SetResolution(
    cfg.motor_spec.steps_per_rev,
    tmc5160_test_config::GetEncoderPulsesPerRev<SELECTED_PLATFORM>(),
    tmc5160_test_config::GetEncoderInvertDirection<SELECTED_PLATFORM>()
);
```

The helper functions automatically configure:
- **Motor specifications** (steps, current, resistance, inductance) - from MotorConfig
- **Board hardware configuration** (sense resistor, supply voltage, MOSFETs) - from BoardConfig
- **Chopper configuration** (from MotorConfig)
- **StealthChop configuration** (from MotorConfig)
- **Power stage configuration** (from BoardConfig)
- **Mechanical system** (gear ratio from MotorConfig, system type from PlatformConfig)

#### Manual Configuration Approach

For custom configurations or non-ESP32 platforms, configure manually:

```cpp
tmc5160::DriverConfig cfg{};

// Power stage configuration (user-friendly physical parameters)
cfg.power_stage.mosfet_miller_charge_nc = 30.0f;  // MOSFET Miller charge in nC (0 = auto-calculate)
cfg.power_stage.bbm_time_ns = 200;                 // Break-before-make time in nanoseconds (0 = auto-calculate)
cfg.power_stage.sense_filter = tmc5160::SenseFilterTime::T100ns; // Sense amplifier filter time constant
cfg.power_stage.over_temp_protection = tmc5160::OverTempProtection::Temp150C; // Over-temperature protection (150°C threshold)

// Short protection (user-friendly voltage thresholds and timing)
cfg.power_stage.s2vs_voltage_mv = 625;  // Short to VS voltage threshold in mV (0 = auto = 625mV)
cfg.power_stage.s2g_voltage_mv = 625;  // Short to GND voltage threshold in mV (0 = auto = 625mV)
cfg.power_stage.shortfilter = 1;        // Spike filter bandwidth (0=100ns, 1=1µs, 2=2µs, 3=3µs)
cfg.power_stage.short_detection_delay_us_x10 = 0;  // Detection delay in 0.1µs units (0 = auto = 0.85µs)

// Motor specification (user-friendly physical parameters)
// REQUIRED for automatic current calculation:
cfg.motor_spec.steps_per_rev = 200;
cfg.motor_spec.rated_current_ma = 1680;
cfg.motor_spec.sense_resistor_mohm = 50;  // 0.05Ω (REQUIRED for calculation)
cfg.motor_spec.supply_voltage_mv = 24000;  // 24V (REQUIRED for calculation)

// Optional motor specifications:
cfg.motor_spec.rated_voltage_mv = 12000;  // Motor rated voltage
cfg.motor_spec.winding_resistance_mohm = 3000;  // Winding resistance (for StealthChop validation)
cfg.motor_spec.winding_inductance_uh = 2800;  // Winding inductance

// Desired current settings (0 = use rated_current_ma and auto-calculate)
cfg.motor_spec.run_current_ma = 0;   // 0 = use rated_current_ma
cfg.motor_spec.hold_current_ma = 0;  // 0 = auto-calculate as 30% of run

// Optional: Percentage adjustments for calculated current settings
cfg.motor_spec.scaler_adjustment_percent = 0.0f;  // Adjust GLOBAL_SCALER by percentage
cfg.motor_spec.irun_adjustment_percent = 0.0f;    // Adjust IRUN by percentage
cfg.motor_spec.ihold_adjustment_percent = 0.0f;   // Adjust IHOLD by percentage

// Note: IRUN, IHOLD, and GLOBAL_SCALER are automatically calculated during initialization
// DO NOT set them manually - they are calculated from motor_spec parameters

// Chopper configuration (SpreadCycle mode - recommended)
cfg.chopper.mode = tmc5160::ChopperMode::SPREAD_CYCLE;  // SpreadCycle mode (recommended)
cfg.chopper.toff = 5;              // Off time (0-15, 0=disabled, 5=typical for 16-30kHz)
cfg.chopper.tbl = static_cast<uint8_t>(tmc5160::ChopperBlankTime::TBL_36CLK);  // Blank time (36 clocks, typical)
cfg.chopper.hstrt = 4;             // Hysteresis start (0-7, 4=typical)
cfg.chopper.hend = 0;              // Hysteresis end (0-15 encoded, 0=typical)
cfg.chopper.tpfd = 0;              // Passive fast decay (0=disabled, increase if resonances)
cfg.chopper.mres = static_cast<uint8_t>(tmc5160::MicrostepResolution::MRES_16);  // 16 microsteps (typical)
cfg.chopper.intpol = true;         // Enable interpolation to 256 microsteps (recommended)
cfg.chopper.dedge = false;         // Double edge step pulses (typically false)

// Alternative: Classic mode (requires more tuning)
// cfg.chopper.mode = tmc5160::ChopperMode::CLASSIC;
// cfg.chopper.tfd = 5;              // Fast decay time (similar to toff)
// cfg.chopper.hend = 4;             // Sine wave offset (positive offset for zero crossing)
// cfg.chopper.disfdcc = false;      // Enable comparator termination

// StealthChop configuration (automatic tuning mode - recommended)
cfg.stealthchop.pwm_freq = static_cast<uint8_t>(tmc5160::StealthChopPwmFreq::PWM_FREQ_1);  // ~35kHz @ 12MHz
cfg.stealthchop.pwm_ofs = 30;      // Initial PWM offset (normal mode, will be optimized by AT#1)
cfg.stealthchop.pwm_grad = 0;      // Initial PWM gradient (will be optimized by AT#2)
cfg.stealthchop.pwm_autoscale = true;  // Enable automatic current scaling (recommended)
cfg.stealthchop.pwm_autograd = true;   // Enable automatic gradient adaptation (recommended)
cfg.stealthchop.pwm_reg = static_cast<uint8_t>(tmc5160::StealthChopRegulationSpeed::MODERATE);  // Balanced regulation
cfg.stealthchop.pwm_lim = static_cast<uint8_t>(tmc5160::StealthChopJerkReduction::MODERATE);   // Balanced jerk reduction
cfg.stealthchop.freewheel = tmc5160::PWMFreewheel::NORMAL;  // Freewheeling mode

// Alternative: Use helper constructor with enums (most intuitive)
// tmc5160::StealthChopConfig stealth(
//     tmc5160::StealthChopPwmFreq::PWM_FREQ_1,
//     tmc5160::StealthChopRegulationSpeed::MODERATE,
//     tmc5160::StealthChopJerkReduction::MODERATE
// );
// cfg.stealthchop = stealth;


// Motor direction
cfg.direction = tmc5160::MotorDirection::NORMAL;

// Global configuration (GCONF register)
cfg.global_config.en_pwm_mode = true;  // Enable stealthChop
cfg.global_config.multistep_filt = true; // Enable step filtering
cfg.global_config.shaft = false;  // Normal direction

// Ramp generator configuration
// Specify units for velocity and acceleration parameters (critical for proper conversion)
cfg.ramp_config.velocity_unit = tmc5160::Unit::Steps;      // All velocities in steps/s
cfg.ramp_config.acceleration_unit = tmc5160::Unit::Steps;   // All accelerations in steps/s²

// Or use physical units (requires mechanical system configuration):
// cfg.ramp_config.velocity_unit = tmc5160::Unit::RPM;      // Velocities in RPM
// cfg.ramp_config.acceleration_unit = tmc5160::Unit::Deg;  // Accelerations in deg/s²

cfg.ramp_config.vstart = 0.0f;      // Start velocity (unit specified by velocity_unit, 0 = can be zero)
cfg.ramp_config.vstop = 10.0f;     // Stop velocity (unit specified by velocity_unit, must be >= VSTART)
cfg.ramp_config.vmax = 1000.0f;     // Maximum velocity (unit specified by velocity_unit, set before motion)
cfg.ramp_config.v1 = 0.0f;          // Transition velocity (unit specified by velocity_unit, 0 = disabled)
cfg.ramp_config.amax = 500.0f;      // Maximum acceleration (unit specified by acceleration_unit, set before motion)
cfg.ramp_config.a1 = 0.0f;          // First acceleration (unit specified by acceleration_unit, 0 = use AMAX)
cfg.ramp_config.dmax = 0.0f;        // Maximum deceleration (unit specified by acceleration_unit, 0 = uses AMAX)
cfg.ramp_config.d1 = 100.0f;        // First deceleration (unit specified by acceleration_unit, must not be 0)
cfg.ramp_config.tpowerdown_ms = 437.0f;  // Power down delay in milliseconds (~0.44s at 12MHz, range: 0-5600ms)
cfg.ramp_config.tzerowait_ms = 0.0f;     // Zero wait time in milliseconds (no delay, range: 0-2000ms)

// Clock frequency
cfg.f_clk = 12000000;  // 12 MHz (default)

driver.Initialize(cfg);
```

## Configuration Methods

### Ramp Control Configuration

```cpp
// Set ramp mode
driver.rampControl.SetRampMode(tmc5160::RampMode::POSITIONING);

// Set motion parameters (unit-aware API)
driver.rampControl.SetTargetPosition(1000.0f, tmc5160::Unit::Steps);
driver.rampControl.SetMaxSpeed(1000.0f, tmc5160::Unit::Steps);      // steps/s
driver.rampControl.SetAcceleration(500.0f, tmc5160::Unit::Steps);   // steps/s²
driver.rampControl.SetRampSpeeds(0.0f, 10.0f, 0.0f, tmc5160::Unit::Steps); // start, stop, transition

// Or use physical units (requires mechanical system configuration)
driver.rampControl.SetMaxSpeed(100.0f, tmc5160::Unit::RPM);         // 100 RPM
driver.rampControl.SetAcceleration(50.0f, tmc5160::Unit::Deg);     // 50 deg/s²
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

// Configure StealthChop (automatic tuning mode - recommended)
tmc5160::StealthChopConfig stealth_cfg{};
stealth_cfg.pwm_freq = 1;           // PWM frequency (~35kHz @ 12MHz)
stealth_cfg.pwm_ofs = 30;          // Initial PWM offset (optimized by AT#1)
stealth_cfg.pwm_grad = 0;          // Initial PWM gradient (optimized by AT#2)
stealth_cfg.pwm_autoscale = true;  // Enable automatic current scaling
stealth_cfg.pwm_autograd = true;   // Enable automatic gradient adaptation
stealth_cfg.pwm_reg = 4;           // Regulation coefficient (balanced)
stealth_cfg.pwm_lim = 12;          // Amplitude limit (default)
driver.motorControl.ConfigureStealthChop(stealth_cfg);

// IMPORTANT: Motor must be at standstill when StealthChop is first enabled
// Keep motor stopped for at least 128 chopper periods after enabling

// Set mode change speeds (velocity thresholds)
driver.motorControl.SetModeChangeSpeeds(100.0f, 500.0f, 2000.0f, tmc5160::Unit::Steps);
// pwm_thrs: StealthChop threshold (below this: StealthChop, above: SpreadCycle)
// cool_thrs: CoolStep threshold (below this: CoolStep disabled)
// high_thrs: High-speed mode threshold
```

### Encoder Configuration

```cpp
tmc5160::EncoderConfig enc_cfg{};
// N channel configuration
enc_cfg.n_channel_active = tmc5160::ReferenceSwitchActiveLevel::ACTIVE_HIGH;
enc_cfg.n_sensitivity = tmc5160::EncoderNSensitivity::RISING_EDGE;  // Trigger on rising edge
enc_cfg.ignore_ab_polarity = true;  // Ignore A/B polarity for N events

// Clear/latch mode
enc_cfg.clear_mode = tmc5160::EncoderClearMode::ONCE;  // Latch on next N event
enc_cfg.clear_enc_x_on_event = false;  // Latch only, don't clear counter
enc_cfg.latch_xactual_with_enc = true;  // Also latch XACTUAL position

// Prescaler mode
enc_cfg.prescaler_mode = tmc5160::EncoderPrescalerMode::BINARY;  // Binary mode

driver.encoder.Configure(enc_cfg);

// Set encoder resolution (motor steps per encoder resolution)
// Automatically calculates ENC_CONST and selects binary/decimal mode
driver.encoder.SetResolution(200, 1000, false); // 200 steps/rev, 1000 pulses/rev

// Set allowed deviation threshold (in steps)
driver.encoder.SetAllowedDeviation(10); // 10 steps tolerance
```

### StallGuard2 Configuration

StallGuard2 provides accurate measurement of motor load and can detect stalls. Used for sensorless homing, CoolStep, and diagnostics.

**Prerequisites**: StallGuard2 requires SpreadCycle mode (StealthChop disabled).

```cpp
// IMPORTANT: Enable SpreadCycle mode first
driver.motorControl.SetStealthChopEnabled(false);

// Configure StallGuard2 with user-friendly API
tmc5160::StallGuardConfig sg_cfg{};

// Set threshold (lower = more sensitive, higher = less sensitive)
sg_cfg.threshold = 0;  // Starting value, works with most motors

// Enable filter for smoother readings (recommended for CoolStep)
sg_cfg.enable_filter = true;

// Set velocity thresholds (StallGuard2 only active between these speeds)
sg_cfg.min_velocity = 500.0f;   // Enable StallGuard2 above 500 steps/s
sg_cfg.max_velocity = 5000.0f;  // Disable StallGuard2 above 5000 steps/s (0 = no limit)
sg_cfg.velocity_unit = tmc5160::Unit::Steps;

// Stop motor when stall detected (for sensorless homing)
sg_cfg.stop_on_stall = true;

// Configure StallGuard2 (automatically sets velocity thresholds and stop on stall)
driver.diagnostics.ConfigureStallGuard(sg_cfg);

// Alternative: Use sensitivity enum for convenience
// tmc5160::StallGuardConfig sg_cfg(tmc5160::StallGuardSensitivity::MODERATE, true, 500.0f, 5000.0f, tmc5160::Unit::Steps, true);
```

**Note**: `semin`, `semax`, `seup`, `sedn`, `seimin` are CoolStep parameters, not StallGuard2. Configure CoolStep separately if needed.

**For detailed tuning guide, examples, and best practices, see**: [Advanced Configuration - StallGuard2](../docs/special_features_advanced_configuration.md#stallguard2-load-measurement)

### CoolStep Configuration

CoolStep automatically reduces motor current when load is low, saving energy and reducing heat.

**Prerequisites**: CoolStep requires SpreadCycle mode (StealthChop disabled) and properly tuned StallGuard2.

```cpp
// IMPORTANT: Enable SpreadCycle mode first
driver.motorControl.SetStealthChopEnabled(false);

// Configure CoolStep with user-friendly API
tmc5160::CoolStepConfig coolstep{};

// Set thresholds using actual SG values (0-1023)
coolstep.lower_threshold_sg = 64;   // Increase current when SG < 64
coolstep.upper_threshold_sg = 256;  // Decrease current when SG >= 256

// Configure response speeds using enums
coolstep.increment_step = tmc5160::CoolStepIncrementStep::STEP_2;  // Moderate response
coolstep.decrement_speed = tmc5160::CoolStepDecrementSpeed::EVERY_8;  // Stable reduction

// Minimum current: 50% of IRUN
coolstep.min_current = tmc5160::CoolStepMinCurrent::HALF_IRUN;

// Enable filter for smoother operation
coolstep.enable_filter = true;

// Set velocity thresholds (CoolStep only active between these speeds)
coolstep.min_velocity = 500.0f;   // Enable CoolStep above 500 steps/s
coolstep.max_velocity = 5000.0f;  // Disable CoolStep above 5000 steps/s
coolstep.velocity_unit = tmc5160::Unit::Steps;

// Configure CoolStep (automatically sets velocity thresholds)
driver.motorControl.ConfigureCoolStep(coolstep);
```

**For detailed tuning guide, examples, and best practices, see**: [Advanced Configuration - CoolStep](../docs/special_features_advanced_configuration.md#coolstep-current-reduction)

### DcStep Configuration

DcStep allows the motor to run near its load limit without losing steps by automatically reducing velocity when overloaded.

**Prerequisites**: DcStep requires SD_MODE=1 (external step/dir mode) or can be enabled via VDCMIN threshold. CHOPCONF.TOFF should be >2 (preferably 8-15).

```cpp
// Configure DcStep with user-friendly API
tmc5160::DcStepConfig dcstep{};

// Set minimum velocity threshold (with unit support)
dcstep.min_velocity = 1000.0f;   // Enable DcStep above 1000 steps/s
dcstep.velocity_unit = tmc5160::Unit::Steps;

// Auto-calculate PWM on-time from blank time (recommended)
dcstep.pwm_on_time_us = 0.0f;  // 0 = auto-calculate

// Moderate stall detection sensitivity (recommended)
dcstep.stall_sensitivity = tmc5160::DcStepStallSensitivity::MODERATE;

// Don't stop on stall (continue operation)
dcstep.stop_on_stall = false;

// Configure DcStep (automatically sets CHOPCONF.vhighfs and CHOPCONF.vhighchm)
driver.motorControl.ConfigureDcStep(dcstep);
```

**For detailed tuning guide, examples, and best practices, see**: [Advanced Configuration - DcStep](../docs/special_features_advanced_configuration.md#dcstep-automatic-commutation)

## Default Values

| Option | Default | Description |
|--------|---------|-------------|
| `power_stage.mosfet_miller_charge_nc` | 10.0f | MOSFET Miller charge in nC (0 = auto-calculate, auto-calculates DRVSTRENGTH) |
| `power_stage.bbm_time_ns` | 100 | Break-before-make time in nanoseconds (0 = auto-calculate, auto-calculates BBMTIME/BBMCLKS) |
| `power_stage.sense_filter` | T100ns | Sense amplifier filter time constant enum |
| `power_stage.over_temp_protection` | Temp150C | Over-temperature protection (150°C threshold) |
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
| `ramp_config.velocity_unit` | Steps | Unit for all velocity parameters (Steps, Rad, Deg, Mm, RPM) |
| `ramp_config.acceleration_unit` | Steps | Unit for all acceleration parameters (Steps, Rad, Deg, Mm) |
| `ramp_config.vstart` | 0.0 | Start velocity (unit specified by velocity_unit) |
| `ramp_config.vstop` | 10.0 | Stop velocity (unit specified by velocity_unit, must be >= VSTART) |
| `ramp_config.vmax` | 0.0 | Maximum velocity (unit specified by velocity_unit, must be set before motion) |
| `ramp_config.v1` | 0.0 | Transition velocity (unit specified by velocity_unit, 0 = disabled) |
| `ramp_config.amax` | 0.0 | Maximum acceleration (unit specified by acceleration_unit, must be set before motion) |
| `ramp_config.a1` | 0.0 | First acceleration (unit specified by acceleration_unit, 0 = use AMAX) |
| `ramp_config.dmax` | 0.0 | Maximum deceleration (unit specified by acceleration_unit, 0 = uses AMAX) |
| `ramp_config.d1` | 100.0 | First deceleration (unit specified by acceleration_unit, must not be 0) |
| `ramp_config.tpowerdown_ms` | 437.0 | Power down delay in milliseconds (~0.44s at 12MHz, range: 0-5600ms) |
| `ramp_config.tzerowait_ms` | 0.0 | Zero wait time in milliseconds (no delay, range: 0-2000ms) |
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
cfg.chopper.mode = tmc5160::ChopperMode::SPREAD_CYCLE;  // SpreadCycle mode (recommended)
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

