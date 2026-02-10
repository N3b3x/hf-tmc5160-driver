# Driver Configuration Guide

## Overview

The TMC51x0 driver (TMC5130 & TMC5160) library uses a **compile-time configuration** approach with struct-based configuration using `static constexpr` members for zero runtime overhead. The configuration system has four distinct layers:
1. **MotorConfig**: Motor-specific parameters (physical specs, chopper, StealthChop)
2. **BoardConfig**: Board-specific hardware parameters (sense resistor, MOSFETs, clock)
3. **PlatformConfig**: Platform/application-specific parameters (reference switches, encoder, mechanical system)
4. **TestConfig**: Test-specific parameters (StallGuard, motion profiles)

All configurations are defined as structs in `esp32_tmc51x0_test_config.hpp` and automatically converted to driver register values during initialization.

## Configuration Hierarchy

### 1. BoardConfig (Board-Specific Hardware)

**Board-specific**: These parameters depend on the TMC51x0 driver (TMC5130 & TMC5160) board hardware and stay the same regardless of which motor is connected or what platform it's used on.

- Sense resistor value (e.g., 50 mOhm for 0.05Ω)
- Supply voltage (e.g., 24000 mV for 24V)
- Clock frequency (e.g., 12000000 Hz for 12 MHz)
- Power stage MOSFET characteristics (Miller charge, BBM time)
- Short protection defaults

**Location**: `BoardConfig_*` structs in `esp32_tmc51x0_test_config.hpp`

### 2. MotorConfig (Motor-Specific)

**Motor-specific**: These parameters depend on the motor being used and how it should be driven.

- Motor physical specs (steps/rev, rated current, resistance, inductance)
- Chopper settings (TOFF, TBL, HSTRT, HEND)
- StealthChop settings (PWM frequency, offset, autoscale)
- Motor-specific gear ratio (if motor has integrated gearbox)

**Location**: `MotorConfig_*` structs in `esp32_tmc51x0_test_config.hpp`

### 3. PlatformConfig (Platform/Application-Specific)

**Platform-specific**: These parameters depend on how the motor is integrated into the mechanical system and what sensors/switches are used. The same motor can be used on different platforms with different configurations.

- Reference switches (endstops) configuration
- Encoder configuration
- Mechanical system type (DirectDrive, LeadScrew, BeltDrive, Gearbox)
- Lead screw pitch, belt parameters (if applicable)

**Location**: `PlatformConfig_*` structs in `esp32_tmc51x0_test_config.hpp`

## Key Concepts

### Automatic Current Calculation

The driver **automatically calculates** IRUN, IHOLD, and GLOBAL_SCALER from motor physical specifications:
- Motor rated current (`rated_current_ma`)
- Sense resistor value (`sense_resistor_mohm`) - from BoardConfig
- Supply voltage (`supply_voltage_mv`) - from BoardConfig

**You should NOT manually set IRUN, IHOLD, or GLOBAL_SCALER** - these are calculated internally.

### Configuration Flow

1. **Board Configuration** (`BoardConfig` namespace):
   - Sense resistor value (e.g., 50 mOhm for 0.05Ω)
   - Supply voltage (e.g., 24000 mV for 24V)
   - Clock frequency (e.g., 12000000 Hz for 12 MHz)
   - Power stage MOSFET characteristics

2. **Motor Configuration** (Motor config namespaces):
   - Motor physical specs (steps/rev, rated current, resistance, inductance)
   - Chopper settings (TOFF, TBL, HSTRT, HEND)
   - StealthChop settings
   - Motor-specific gear ratio

3. **Platform Configuration** (`PlatformConfig` namespace):
   - Reference switches configuration
   - Encoder configuration
   - Mechanical system type and parameters

4. **Helper Functions**:
   - `ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg)` - Configures motor + board
   - `ConfigureDriverFromMotor_17HS4401S_Direct(cfg)` - Configures motor + board
   - `ConfigureDriverFromMotor_AppliedMotion_5034(cfg)` - Configures motor + board
   - `ApplyPlatformConfig(cfg)` - Applies platform config to DriverConfig
   - `GetReferenceSwitchConfig()` - Gets reference switch config from PlatformConfig
   - `GetEncoderConfig()` - Gets encoder config from PlatformConfig

## Usage Example

```cpp
#include "esp32_tmc51x0_test_config.hpp"

// Recommended: Use test rig selection (automatically configures motor, board, and platform)
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER;

// Configure driver from test rig
tmc51x0::DriverConfig cfg{};
tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
driver.Initialize(cfg);

// Alternative: Manual selection (for advanced use cases)
static constexpr tmc51x0_test_config::MotorType SELECTED_MOTOR = 
    tmc51x0_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;
static constexpr tmc51x0_test_config::BoardType SELECTED_BOARD = 
    tmc51x0_test_config::BoardType::BOARD_TMC51x0_EVAL;
static constexpr tmc51x0_test_config::PlatformType SELECTED_PLATFORM = 
    tmc51x0_test_config::PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG;

// 1. Configure motor
tmc51x0::DriverConfig cfg{};

if constexpr (SELECTED_MOTOR == tmc51x0_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    tmc51x0_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
} else if constexpr (SELECTED_MOTOR == tmc51x0_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    tmc51x0_test_config::ConfigureDriverFromMotor_17HS4401S_Direct(cfg);
} else if constexpr (SELECTED_MOTOR == tmc51x0_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    tmc51x0_test_config::ConfigureDriverFromMotor_AppliedMotion_5034(cfg);
}

// 2. Apply board configuration (sense resistor, supply voltage, MOSFETs, etc.)
tmc51x0_test_config::ApplyBoardConfig<SELECTED_BOARD>(cfg);

// 3. Apply platform configuration (mechanical system type, etc.)
tmc51x0_test_config::ApplyPlatformConfig<SELECTED_PLATFORM>(cfg);

// 4. Initialize driver (current settings calculated automatically)
driver.Initialize(cfg);

// 5. Configure platform-specific features after initialization
// Reference switches
auto ref_cfg = tmc5160_test_config::GetReferenceSwitchConfig<SELECTED_PLATFORM>();
driver.switches.ConfigureReferenceSwitch(ref_cfg);

// Encoder
auto enc_cfg = tmc5160_test_config::GetEncoderConfig<SELECTED_PLATFORM>();
driver.encoder.Configure(enc_cfg);
driver.encoder.SetResolution(
    cfg.motor_spec.steps_per_rev,
    tmc5160_test_config::GetEncoderPulsesPerRev<SELECTED_PLATFORM>(),
    tmc5160_test_config::GetEncoderInvertDirection<SELECTED_PLATFORM>()
);
```

## What Gets Configured Automatically

### Board Configuration (from `BoardConfig`)
- `motor_spec.sense_resistor_mohm`: Sense resistor value (from BoardConfig)
- `motor_spec.supply_voltage_mv`: Supply voltage (from BoardConfig)
- `power_stage.mosfet_miller_charge_nc`: MOSFET Miller charge (from BoardConfig)
- `power_stage.bbm_time_ns`: Break-before-make time (from BoardConfig)
- `power_stage.s2vs_voltage_mv`: Short to VS threshold (from BoardConfig)
- `power_stage.s2g_voltage_mv`: Short to GND threshold (from BoardConfig)
- `f_clk`: Clock frequency (from BoardConfig)

### Motor Specifications (`motor_spec`)
- `steps_per_rev`: Motor full steps per revolution (from MotorConfig)
- `rated_current_ma`: Motor rated current (from MotorConfig, used for calculation)
- `winding_resistance_mohm`: Motor winding resistance (from MotorConfig, if available)
- `winding_inductance_mh`: Motor winding inductance (from MotorConfig, if available)

### Chopper Configuration (`chopper`)
- `mode`: SpreadCycle (recommended) or Classic
- `toff`: Off time setting
- `tbl`: Blank time
- `hstrt`: Hysteresis start
- `hend`: Hysteresis end
- `mres`: Microstep resolution
- `intpol`: Interpolation enable

### StealthChop Configuration (`stealthchop`)
- `pwm_freq`: PWM frequency
- `pwm_ofs`: PWM offset
- `pwm_autoscale`: Automatic scaling enable
- `pwm_autograd`: Automatic gradient enable

### Power Stage Configuration (`power_stage`)
- `mosfet_miller_charge_nc`: MOSFET Miller charge (from PlatformConfig)
- `bbm_time_ns`: Break-before-make time (from PlatformConfig)
- `sense_filter`: Sense amplifier filter time
- `over_temp_protection`: Over-temperature protection level
- `s2vs_voltage_mv`: Short to VS voltage threshold
- `s2g_voltage_mv`: Short to GND voltage threshold

### Mechanical System (`mechanical`)
- `gear_ratio`: Gear ratio (from MotorConfig - motor-specific)
- `system_type`: DirectDrive, Gearbox, LeadScrew, BeltDrive (from PlatformConfig - platform-specific)
- `lead_screw_pitch_mm`: Lead screw pitch (from PlatformConfig, if applicable)
- `belt_pulley_teeth`: Belt pulley teeth (from PlatformConfig, if applicable)
- `belt_pitch_mm`: Belt pitch (from PlatformConfig, if applicable)

### Platform Configuration (configured after initialization)
- **Reference Switches**: Use `GetReferenceSwitchConfig()` to get configuration from `PlatformConfig::ReferenceSwitches`
- **Encoder**: Use `GetEncoderConfig()` to get configuration from `PlatformConfig::Encoder`

## Important Notes

1. **Current Settings**: IRUN, IHOLD, and GLOBAL_SCALER are **calculated automatically** during `Initialize()`. Do not set them manually.

2. **Board Config**: All board-specific values (sense resistor, supply voltage, MOSFET characteristics) are defined in `BoardConfig` namespace and shared across all motors on the same board.

3. **Motor Configs**: Each motor has its own namespace (e.g., `MotorConfig_17HS4401S`) with motor-specific settings (chopper, StealthChop, gear ratio).

4. **Platform Config**: Platform-specific values (reference switches, encoder, mechanical system type) are defined in `PlatformConfig` namespace. These can vary between different platforms/applications even with the same motor.

5. **Configuration Order**:
   - First: Configure motor (via `ConfigureDriverFromMotor_*()`)
   - Second: Apply board config (via `ApplyBoardConfig<SELECTED_BOARD>()`)
   - Third: Apply platform config (via `ApplyPlatformConfig<SELECTED_PLATFORM>()`)
   - Fourth: Initialize driver
   - Fifth: Configure platform-specific features (reference switches, encoder) after initialization

6. **Override Values**: You can override any configuration value after calling the helper functions:
   ```cpp
   ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
   ApplyBoardConfig<SELECTED_BOARD>(cfg);
   ApplyPlatformConfig<SELECTED_PLATFORM>(cfg);
   cfg.chopper.mres = tmc51x0::MicrostepResolution::MRES_16;  // Override microstep resolution
   ```

7. **Adding New Boards/Platforms**: To add a new board or platform:
   - Create a new namespace (e.g., `BoardConfig_OtherBoard` or `PlatformConfig_3DPrinter`)
   - Add a new enum value (e.g., `BOARD_OTHER_BOARD` or `PLATFORM_3D_PRINTER`)
   - Add a new `if constexpr` branch in `ApplyBoardConfig` or `ApplyPlatformConfig` template functions

## Migration from Old Code

### Old Approach (Deprecated)
```cpp
cfg.motor.irun = 20;
cfg.motor.ihold = 10;
cfg.motor.global_scaler = 160;
```

### New Approach (Recommended)
```cpp
// Set motor specifications - current settings calculated automatically
cfg.motor_spec.rated_current_ma = 1680;
cfg.motor_spec.sense_resistor_mohm = 50;
cfg.motor_spec.supply_voltage_mv = 24000;

// Or use helper function (recommended)
ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
```

## Troubleshooting

### Current Settings Not Applied
- Ensure `BoardConfig::SENSE_RESISTOR_MOHM` and `BoardConfig::SUPPLY_VOLTAGE_MV` are set correctly
- Check that motor `rated_current_ma` is set correctly in MotorConfig
- Verify initialization succeeded (`driver.Initialize(cfg)` returns true)

### Wrong Current Values
- Check motor specifications match your actual motor (MotorConfig)
- Verify sense resistor value matches your board hardware (BoardConfig)
- Check supply voltage matches your power supply (BoardConfig)
- Use `scaler_adjustment_percent`, `irun_adjustment_percent`, `ihold_adjustment_percent` for fine-tuning

### Configuration Not Working
- Ensure you're using the correct helper function for your motor
- Check that `SELECTED_MOTOR` matches your motor type
- Verify all required fields are set in `motor_spec`
- Ensure `ApplyPlatformConfig()` is called after motor configuration
- Verify platform-specific features (reference switches, encoder) are configured after initialization

### Reference Switches/Encoder Not Working
- Ensure `GetReferenceSwitchConfig()` or `GetEncoderConfig()` are called after initialization
- Verify platform configuration matches your hardware setup
- Check that platform-specific features are configured via driver methods, not just in DriverConfig

