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

### Motor Selection Method

Motor selection is done via a `static constexpr` variable at the top of your example/test file:

```cpp
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;
```

Change to your desired motor:

```cpp
// For direct drive 17HS4401S:
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_17HS4401S_DIRECT;

// For Applied Motion 5034-369:
static constexpr tmc5160_test_config::MotorType SELECTED_MOTOR = 
    tmc5160_test_config::MotorType::MOTOR_APPLIED_MOTION_5034;
```

Then use `if constexpr` blocks to select the motor configuration namespace:

```cpp
if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_GEARBOX) {
    namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S;
    // Use Motor::IRUN, Motor::IHOLD, etc.
} else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_17HS4401S_DIRECT) {
    namespace Motor = tmc5160_test_config::MotorConfig_17HS4401S_Direct;
    // Use Motor::IRUN, Motor::IHOLD, etc.
} else if constexpr (SELECTED_MOTOR == tmc5160_test_config::MotorType::MOTOR_APPLIED_MOTION_5034) {
    namespace Motor = tmc5160_test_config::MotorConfig_AppliedMotion_5034_369;
    // Use Motor::IRUN, Motor::IHOLD, etc.
}
```

---

## Motor Configuration Namespaces

Each motor has its own configuration namespace in `esp32_tmc5160_test_config.hpp`:

- `MotorConfig_17HS4401S` - Geared version
- `MotorConfig_17HS4401S_Direct` - Direct drive version
- `MotorConfig_AppliedMotion_5034_369` - Applied Motion NEMA 34

These namespaces contain:
- Physical motor specifications
- Driver current settings (IRUN, IHOLD, GLOBAL_SCALER)
- Chopper configuration (TOFF, HEND, HSTRT, TBL)
- StealthChop settings
- Power stage configuration

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

- [esp32_tmc5160_test_config.hpp](../../main/esp32_tmc5160_test_config.hpp) - Full configuration definitions
- [Sinusoidal Example](sinusoidal.md) - Example using motor selection
- [Bounds Finding Example](bounds_finding_sinuous_motion.md) - Example using motor selection

