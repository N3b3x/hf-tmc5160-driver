# Motor Configuration Guide

This guide explains the motor configurations available for ESP32 examples and tests, and how to select them.

## Available Motor Configurations

### 1. MOTOR_17HS4401S_GEARBOX (Default)

**Model**: 17HS4401S-PG518 with 5.18:1 Planetary Gearbox

**Specifications**:
- Rated Current: 1.68A RMS per phase
- Step Angle: 1.8° (200 steps/rev motor, ~1036 steps/rev output)
- Holding Torque: 40Ncm (motor), ~207Ncm (output with gearbox)
- Gear Ratio: 5.18:1 (Planetary)
- Output Steps/Rev: ~1036 steps

**Driver Settings**:
- IRUN: 20 (~1.88A RMS, ~2.66A Peak)
- IHOLD: 10 (~0.94A RMS, ~1.33A Peak)
- GLOBAL_SCALER: 160
- Microsteps: 256 (MRES=0)
- StealthChop: Enabled with auto-calibration

**Use Cases**:
- Applications requiring high torque and precision positioning
- Systems needing smooth, quiet operation
- High-resolution positioning applications

**Power Supply**: 2-3A recommended

---

### 2. MOTOR_17HS4401S_DIRECT

**Model**: 17HS4401S Direct Drive (No Gearbox)

**Specifications**:
- Rated Current: 1.68A RMS per phase
- Step Angle: 1.8° (200 steps/rev)
- Holding Torque: 40Ncm
- Gear Ratio: 1.0:1 (Direct Drive)
- Output Steps/Rev: 200 steps

**Driver Settings**:
- IRUN: 20 (~1.88A RMS, ~2.66A Peak)
- IHOLD: 10 (~0.94A RMS, ~1.33A Peak)
- GLOBAL_SCALER: 160
- Microsteps: 256 (MRES=0)
- StealthChop: Enabled with auto-calibration

**Use Cases**:
- Applications requiring higher speed and lower torque
- Direct drive systems
- Systems where gearbox backlash is undesirable

**Power Supply**: 2-3A recommended

---

### 3. MOTOR_APPLIED_MOTION_5034

**Model**: Applied Motion Products 5034-369 NEMA 34

**Specifications**:
- Rated Current: 4.17A RMS per phase (Bipolar Series)
- Step Angle: 1.8° (200 steps/rev)
- Holding Torque: 636 oz-in (4.5 Nm)
- Resistance: 0.84 Ohms/phase (Bipolar Series)
- Inductance: 10.4 mH/phase (Bipolar Series)
- Gear Ratio: 1.0:1 (Direct Drive)
- Output Steps/Rev: 200 steps

**Driver Settings**:
- IRUN: 28 (~4.17A RMS, ~5.9A Peak)
- IHOLD: 14 (~2.15A RMS, ~3.04A Peak)
- GLOBAL_SCALER: 256 (Full scale)
- Microsteps: 256 (MRES=0)
- StealthChop: Enabled with auto-calibration

**Use Cases**:
- High-torque applications requiring 4A+ current
- Heavy-duty industrial applications
- Systems requiring maximum holding torque

**Power Supply**: **5A+ continuous current required**

**WARNING**: This motor requires significantly higher current than NEMA 17 motors. Ensure your power supply can deliver adequate current.

---

## Current Calculation Details

All configurations use 0.05Ω sense resistors. Current is calculated using the TMC5160 datasheet formula:

```
I_RMS = (GLOBAL_SCALER/256) * ((IRUN+1)/32) * (VFS/RSENSE) * (1/√2)
```

Where:
- VFS = 0.325V (typical full-scale voltage)
- RSENSE = 0.05Ω

### Example Calculation (17HS4401S with IRUN=20, GLOBAL_SCALER=160):

```
I_RMS = (160/256) * ((20+1)/32) * (0.325/0.05) * 0.707
I_RMS = 0.625 * 0.65625 * 6.5 * 0.707
I_RMS ≈ 1.88A RMS
I_Peak ≈ 2.66A Peak
```

---

## How to Select a Motor

### Test Rig Selection Method (Recommended)

Most examples use the `TestRigConfig` template which provides unified access to all configurations:

```cpp
// Select test rig at compile time - automatically selects motor, board, and platform
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER;

// Configure driver from test rig (automatically configures everything)
tmc51x0::DriverConfig cfg{};
tmc51x0_test_config::TestRigConfig<SELECTED_TEST_RIG>::ConfigureDriver(cfg);
// Or use convenience function:
tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
driver.Initialize(cfg);

// Access configuration values
using Config = tmc51x0_test_config::TestRigConfig<SELECTED_TEST_RIG>;
float speed = Config::Test::Motion::BOUNDS_SEARCH_SPEED_RPM;
uint16_t steps = Config::GetMotorOutputFullSteps();
```

Available test rigs:
- **TEST_RIG_CORE_DRIVER**: 17HS4401S motor (geared or direct), TMC51x0 EVAL board, reference switches, encoder
- **TEST_RIG_FATIGUE**: Applied Motion 5034-369 NEMA 34 motor, TMC51x0 EVAL board, reference switches, encoder

### Direct Struct Access

You can also access motor configuration structs directly:

```cpp
// Access motor config values directly
uint16_t steps = MotorConfig_17HS4401S::OUTPUT_FULL_STEPS;
float gear_ratio = MotorConfig_17HS4401S::GEAR_RATIO;
uint16_t current = MotorConfig_17HS4401S::TARGET_RUN_CURRENT_MA;
```

### Manual Motor Selection Method (Advanced)

For advanced use cases, you can manually configure:

```cpp
tmc51x0::DriverConfig cfg{};
tmc51x0_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
tmc51x0_test_config::ApplyBoardConfig<tmc51x0_test_config::BoardType::BOARD_TMC51x0_EVAL>(cfg);
tmc51x0_test_config::ApplyPlatformConfig<tmc51x0_test_config::PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG>(cfg);
driver.Initialize(cfg);
```

---

## Motor Configuration Structs

Each motor has its own configuration struct in `esp32_tmc51x0_test_config.hpp`:

- `MotorConfig_17HS4401S` - Geared version (struct with static constexpr members)
- `MotorConfig_17HS4401S_Direct` - Direct drive version (struct with static constexpr members)
- `MotorConfig_AppliedMotion_5034_369` - Applied Motion NEMA 34

These structs contain `static constexpr` members for:
- Physical motor specifications
- Target current settings (automatically calculated to IRUN, IHOLD, GLOBAL_SCALER)
- Chopper configuration (TOFF, HEND, HSTRT, TBL)
- StealthChop settings
- Ramp profile defaults

## Test Rig Configuration

The project uses two test rigs, each with a specific motor, board, and platform configuration:

1. **Core Driver Test Rig** (`TEST_RIG_CORE_DRIVER`):
   - Motor: 17HS4401S motor with gearbox (default) or direct drive
   - Board: TMC51x0 Evaluation Kit (BOARD_TMC51x0_EVAL)
   - Platform: Core Driver Test Rig (PLATFORM_CORE_DRIVER_TEST_RIG)
   - Features: Reference switches, encoder (AS5047U), gearbox or direct drive
   - Used for: Most comprehensive tests, core driver functionality

2. **Fatigue Test Rig** (`TEST_RIG_FATIGUE`):
   - Motor: Applied Motion 5034-369 motor (MOTOR_APPLIED_MOTION_5034)
   - Board: TMC51x0 Evaluation Kit (BOARD_TMC51x0_EVAL)
   - Platform: Fatigue Test Rig (PLATFORM_FATIGUE_TEST_RIG)
   - Features: Reference switches, encoder (AS5047U), direct drive (NEMA 34)
   - Used for: Bounds finding and point-to-point fatigue motion (fatigue_test_espnow_unit, internal_ramp_sinusoidal)

---

## Important Notes

1. **Motor Selection is Compile-Time Only**: Changing motors requires recompiling the firmware.

2. **Power Supply Requirements**: Ensure your power supply can handle the selected motor's current requirements:
   - 17HS4401S motors: 2-3A recommended
   - Applied Motion 5034-369: 5A+ required

3. **Wiring Verification**: After changing motors, verify:
   - Motor wiring matches selected motor specifications
   - Power supply capacity is adequate
   - Sense resistor values match (0.05Ω assumed)

4. **StealthChop Calibration**: All motors use StealthChop with auto-calibration. The motor will automatically calibrate during operation:
   - AT#1: Standstill calibration (130ms+ at standstill)
   - AT#2: Motion calibration (move at 60-300 RPM for ~400 fullsteps)

5. **Gearbox Considerations**: 
   - Geared motors provide higher torque but lower speed
   - Direct drive motors provide higher speed but lower torque
   - Output resolution differs: geared motor has ~1036 steps/rev output vs 200 steps/rev for direct drive

---

## Troubleshooting

### Motor Not Moving

1. Check power supply voltage (12-36V required)
2. Verify motor current settings match your motor
3. Check StealthChop calibration status (may need time to calibrate)
4. Verify EN pin is properly connected and enabled

### Incorrect Current

1. Verify sense resistor value (0.05Ω assumed)
2. Check IRUN/IHOLD values match your motor rating
3. Verify GLOBAL_SCALER setting

### Motor Overheating

1. Reduce IRUN value
2. Check motor wiring (phases may be swapped)
3. Verify motor is not mechanically overloaded
4. Check power supply voltage is within range

---

## See Also

- [esp32_tmc51x0_test_config.hpp](../main/test_config/esp32_tmc51x0_test_config.hpp) - Full configuration definitions
- [Internal Ramp Sinusoidal Example](internal_ramp_sinusoidal.md) - Example using test rig selection
- [Fatigue Testing](fatigue_test.md) - Point-to-point motion between bounds with ESP-NOW control and bounds finding

