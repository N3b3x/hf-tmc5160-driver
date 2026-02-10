---
layout: default
title: "Examples"
description: "Complete example walkthroughs for the TMC51x0 driver (TMC5130 & TMC5160)"
nav_order: 7
parent: "📚 Documentation"
permalink: /docs/examples/
---

# Examples

This guide walks through progressively more advanced uses of the TMC51x0 driver.
Each example builds on the previous ones, so working through them in order is recommended.

> **Prerequisites** -- Before diving in, make sure you have:
> - The driver [installed](installation.md) and your [hardware wired](hardware_setup.md)
> - A working [SPI or UART interface](platform_integration.md) (`MySPI` class in these examples)
> - Read the [Quick Start](quickstart.md) for the `Result<T>` error handling pattern

> **Error handling note:** These examples omit most `Result` checks for readability.
> In production code, always check return values -- especially `Initialize()`, `Enable()`,
> and any call that communicates with the TMC chip. See the
> [Error Handling section in the README](../README.md#error-handling----the-resultt-pattern)
> for the full pattern, or [`inc/tmc51x0_result.hpp`](../inc/tmc51x0_result.hpp) for all error codes.

---

## Example 1: Basic Positioning Mode

**Difficulty:** Beginner | **What you learn:** Initialization, positioning mode, polling for completion

Move the motor to a specific angle and wait until it arrives.

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi;                                  // Your platform SPI (see platform_integration.md)
tmc51x0::TMC51x0<MySPI> driver(spi);

int main() {
    // 1. Initialize -- provide motor specs; IRUN/IHOLD/GLOBAL_SCALER are calculated for you
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.rated_current_ma = 1500;        // Motor rated current (from datasheet)
    cfg.motor_spec.sense_resistor_mohm = 50;        // Sense resistor on your board (mOhm)
    cfg.motor_spec.supply_voltage_mv = 24000;       // Motor supply voltage (mV)
    if (!driver.Initialize(cfg)) { return -1; }     // Check -- this can fail on SPI errors

    // 2. Enable motor output, then configure motion profile
    driver.motorControl.Enable();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetMaxSpeed(60.0f, tmc51x0::Unit::RPM);          // Peak speed
    driver.rampControl.SetAcceleration(5.0f, tmc51x0::Unit::RevPerSec); // Ramp up
    driver.rampControl.SetDeceleration(5.0f, tmc51x0::Unit::RevPerSec); // Ramp down

    // 3. Command a move -- the ramp generator handles accel/decel automatically
    driver.rampControl.SetTargetPosition(180.0f, tmc51x0::Unit::Deg);

    // 4. Poll until the internal ramp generator reports "target reached"
    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached.IsOk() && reached.Value()) break;
        // In FreeRTOS: vTaskDelay(pdMS_TO_TICKS(10));
    }

    return 0;
}
```

### What's happening

| Step | What the driver does internally |
|------|---------------------------------|
| `Initialize(cfg)` | Resets the TMC chip, writes GCONF, calculates and writes IRUN/IHOLD/GLOBAL_SCALER from your motor specs, configures the chopper and StealthChop defaults |
| `Enable()` | Pulls the hardware enable pin (DRV_ENN) active, powering the motor coils |
| `SetRampMode(POSITIONING)` | Writes RAMPMODE=0 -- the internal ramp generator will decelerate and stop at the target |
| `SetTargetPosition(180°)` | Converts 180° to microsteps (using steps_per_rev and MRES) and writes XTARGET |
| `IsTargetReached()` | Reads the `position_reached` flag from RAMP_STAT -- returns `Result<bool>` |

**Related source:** [`internal_ramp_sinusoidal.cpp`](../examples/esp32/main/internal_ramp_test_suites/internal_ramp_sinusoidal.cpp) -- a full ESP32 example using positioning mode with `BackAndForthMotion`

---

## Example 2: Velocity Mode

**Difficulty:** Beginner | **What you learn:** Continuous rotation, direction control, stopping

Run the motor at a constant speed -- useful for conveyors, fans, or any continuous rotation.

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi;
tmc51x0::TMC51x0<MySPI> driver(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    if (!driver.Initialize(cfg)) { return -1; }

    driver.motorControl.Enable();

    // Velocity mode: motor accelerates to target speed and holds it
    driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_POS);    // Positive direction
    driver.rampControl.SetMaxSpeed(30.0f, tmc51x0::Unit::RPM);
    driver.rampControl.SetAcceleration(2.0f, tmc51x0::Unit::RevPerSec);

    // Motor now runs continuously at 30 RPM ...

    // Reverse direction:
    // driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_NEG);

    // Decelerate to stop:
    // driver.rampControl.Stop();

    return 0;
}
```

### Key differences from positioning mode

- **VELOCITY_POS / VELOCITY_NEG**: Motor runs indefinitely until you call `Stop()` or change mode
- **No target position**: There is no `IsTargetReached()` event -- you decide when to stop
- **Direction**: Controlled by the ramp mode, not by the sign of the target position

---

## Example 3: StealthChop Silent Operation

**Difficulty:** Intermediate | **What you learn:** StealthChop/SpreadCycle mode switching, velocity thresholds

StealthChop provides near-silent motor operation at low speeds by using voltage-mode PWM.
Above a configurable speed threshold, the driver automatically switches to SpreadCycle for better high-speed torque.

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi;
tmc51x0::TMC51x0<MySPI> driver(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    if (!driver.Initialize(cfg)) { return -1; }

    // Enable StealthChop -- motor MUST be at standstill when first enabled
    driver.motorControl.SetStealthChopEnabled(true);

    // Set the crossover speed: below 200 RPM = StealthChop (silent),
    //                          above 200 RPM = SpreadCycle (higher torque)
    driver.thresholds.SetModeChangeSpeeds(
        200.0f,  // TPWMTHRS  -- StealthChop/SpreadCycle boundary
        0.0f,    // TCOOLTHRS -- CoolStep threshold (0 = disabled)
        0.0f,    // THIGH     -- high-speed threshold (0 = disabled)
        tmc51x0::Unit::RPM
    );

    // Move at 30 RPM -- stays in StealthChop (below the 200 RPM threshold)
    driver.motorControl.Enable();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetMaxSpeed(30.0f, tmc51x0::Unit::RPM);
    driver.rampControl.SetAcceleration(2.0f, tmc51x0::Unit::RevPerSec);
    driver.rampControl.SetTargetPosition(90.0f, tmc51x0::Unit::Deg);

    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached.IsOk() && reached.Value()) break;
    }

    return 0;
}
```

### How StealthChop works

1. **AT#1 (standstill tuning):** When StealthChop is first enabled, the driver auto-tunes PWM parameters for ~150 ms while the motor is stationary
2. **AT#2 (motion tuning):** During the first motion, PWM gradient is optimized automatically
3. **Mode switching:** Above TPWMTHRS velocity, the driver seamlessly switches to SpreadCycle

> **Important:** StallGuard2 does **not** work in StealthChop mode. If you need stall detection, either disable StealthChop or set the threshold so the motor is in SpreadCycle during the critical phase.

**Related:** [Advanced Configuration -- StealthChop](special_features_advanced_configuration.md#stealthchop-voltage-pwm-mode)

---

## Example 4: Encoder Closed-Loop Control

**Difficulty:** Intermediate | **What you learn:** Encoder setup, resolution matching, step-loss detection

An ABN incremental encoder provides position feedback. The TMC51x0 compares encoder position against the internal step counter and flags deviations (step loss).

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi;
tmc51x0::TMC51x0<MySPI> driver(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    if (!driver.Initialize(cfg)) { return -1; }

    // --- Encoder setup ---
    tmc51x0::EncoderConfig enc_cfg{};               // Default: ABN mode
    driver.encoder.Configure(enc_cfg);
    driver.encoder.SetResolution(
        200,    // Motor: 200 full steps per revolution (1.8° stepper)
        4096,   // Encoder: 4096 pulses per revolution (PPR)
        false   // Don't invert encoder direction
    );
    driver.encoder.SetAllowedDeviation(100);         // Flag if >100 steps off

    // --- Move and monitor ---
    driver.motorControl.Enable();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetMaxSpeed(30.0f, tmc51x0::Unit::RPM);
    driver.rampControl.SetAcceleration(5.0f, tmc51x0::Unit::RevPerSec);
    driver.rampControl.SetTargetPosition(180.0f, tmc51x0::Unit::Deg);

    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached.IsOk() && reached.Value()) break;

        // Check encoder vs. internal position during motion
        auto dev = driver.encoder.IsDeviationWarning();
        if (dev.IsOk() && dev.Value()) {
            printf("Step loss detected!\n");
            driver.encoder.ClearDeviationWarning();
            // In production: consider reducing speed or increasing current
        }
    }

    // Final check: compare encoder position to expected
    auto enc_pos = driver.encoder.GetPosition();
    if (enc_pos.IsOk()) {
        printf("Encoder position: %d\n", enc_pos.Value());
    }

    return 0;
}
```

### How the deviation check works

The TMC51x0 continuously compares `X_ENC` (encoder position, scaled by `ENC_CONST`) against `XACTUAL` (internal step counter). When the difference exceeds `ENC_DEVIATION`, the `enc_dev_warn` flag is set in `ENC_STATUS`. This flag persists until you read and clear it.

**Related source:** [`abn_encoder_reader.cpp`](../examples/esp32/main/sensors/abn_encoder_reader.cpp) -- continuous encoder position reading on ESP32

---

## Example 5: StallGuard2 Stall Detection

**Difficulty:** Intermediate | **What you learn:** StallGuard2 configuration, load monitoring, stall response

StallGuard2 measures motor load in real time (value 0--1023). A low value means high load; zero means stall. This enables sensorless homing, overload protection, and CoolStep.

```cpp
#include "inc/tmc51x0.hpp"

MySPI spi;
tmc51x0::TMC51x0<MySPI> driver(spi);

int main() {
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.rated_current_ma = 1500;
    cfg.motor_spec.sense_resistor_mohm = 50;
    cfg.motor_spec.supply_voltage_mv = 24000;
    if (!driver.Initialize(cfg)) { return -1; }

    // StallGuard2 ONLY works in SpreadCycle mode (not StealthChop!)
    driver.motorControl.SetStealthChopEnabled(false);

    // Configure StallGuard2 threshold: SGT range is -64 to +63
    //   Lower = more sensitive (detects lighter loads as stalls)
    //   Higher = less sensitive (only detects hard stalls)
    //   Start at 0 and tune from there for your motor + mechanics
    driver.stallGuard.ConfigureStallGuard(0, true);   // SGT=0, filter=on

    // StallGuard2 only reports valid readings above TCOOLTHRS velocity
    driver.thresholds.SetModeChangeSpeeds(0.0f, 10.0f, 0.0f, tmc51x0::Unit::RPM);

    // Run in velocity mode and monitor load
    driver.motorControl.Enable();
    driver.rampControl.SetRampMode(tmc51x0::RampMode::VELOCITY_POS);
    driver.rampControl.SetMaxSpeed(30.0f, tmc51x0::Unit::RPM);
    driver.rampControl.SetAcceleration(5.0f, tmc51x0::Unit::RevPerSec);

    while (true) {
        auto sg = driver.stallGuard.GetStallGuardResult();
        if (sg.IsOk()) {
            uint16_t load = sg.Value();       // 0 = stalled, 1023 = no load
            if (load < 100) {
                driver.rampControl.Stop();
                printf("Stall detected! SG_RESULT=%u\n", load);
                break;
            }
        }
    }

    return 0;
}
```

### Tuning StallGuard2

| SGT value | Effect | Use case |
|-----------|--------|----------|
| -20 to -5 | Very sensitive | Sensorless homing (detect light contact) |
| -5 to +5 | Moderate | General stall protection |
| +5 to +30 | Less sensitive | Avoid false triggers on rough mechanics |

> **Tip:** Use `driver.tuning.AutoTuneStallGuard()` to automatically find the optimal SGT for your motor and velocity. See [Advanced Configuration](special_features_advanced_configuration.md#automatic-tuning-with-comprehensive-velocity-range-analysis).

**Related source:** [`stallguard_tuning.cpp`](../examples/esp32/main/tuning/stallguard_tuning.cpp) -- automatic SGT tuning on ESP32

---

## Example 6: Bounds Finding and Point-to-Point Fatigue Testing

**Difficulty:** Advanced | **What you learn:** Homing subsystem, bounds finding, repetitive motion

This example demonstrates the driver's **bounds finding** and **point-to-point motion** capabilities,
as used in the production fatigue test unit ([`fatigue_test_espnow/main.cpp`](../examples/esp32/main/fatigue_test_espnow/main.cpp)).

### Key concepts

1. **Bounds Finding** -- Uses StallGuard2 to detect mechanical limits in both directions, then homes to center
2. **Point-to-Point Motion** -- Positioning mode moves between two positions repeatedly for fatigue testing
3. **Physical Units** -- All positions and speeds specified in degrees and RPM

### Bounds finding

```cpp
using Homing = tmc51x0::TMC51x0<MySPI>::Homing;

// Configure bounds search parameters
Homing::BoundsOptions opt{};
opt.speed_unit      = tmc51x0::Unit::RPM;
opt.position_unit   = tmc51x0::Unit::Deg;
opt.search_speed    = 30.0f;           // Search at 30 RPM
opt.search_span     = 360.0f;          // Max 360° travel per direction
opt.backoff_distance = 5.0f;           // Back off 5° after hitting a stop
opt.timeout_ms      = 10000;           // 10 s timeout per direction
opt.search_accel    = 5.0f;
opt.accel_unit      = tmc51x0::Unit::RevPerSec;

// Home to the center of whatever range is found
Homing::HomeConfig home{};
home.mode = Homing::HomePlacement::AtCenter;

// Execute -- internally: disables StealthChop, enables StallGuard,
// searches negative, then positive, computes span, sets XACTUAL=0 at center
auto result = driver.homing.FindBounds(Homing::BoundsMethod::StallGuard, opt, home);
if (result.IsOk()) {
    auto bounds = result.Value();
    printf("Bounds: min=%.1f° max=%.1f° span=%.1f°\n",
           bounds.min_position, bounds.max_position, bounds.span);
} else {
    printf("Bounds finding failed: %s\n", result.ErrorMessage());
}
```

### Point-to-point fatigue motion

```cpp
// After bounds finding, oscillate between two positions within the safe range
float half_range = bounds.span * 0.4f;   // Use 80% of total range
float pos_a = -half_range;
float pos_b = +half_range;

driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
driver.rampControl.SetMaxSpeed(60.0f, tmc51x0::Unit::RPM);
driver.rampControl.SetAcceleration(5.0f, tmc51x0::Unit::RevPerSec);
driver.rampControl.SetDeceleration(5.0f, tmc51x0::Unit::RevPerSec);

uint32_t cycle = 0;
bool going_to_b = true;
while (cycle < 10000) {
    float target = going_to_b ? pos_b : pos_a;
    driver.rampControl.SetTargetPosition(target, tmc51x0::Unit::Deg);

    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached.IsOk() && reached.Value()) break;
    }

    going_to_b = !going_to_b;
    if (!going_to_b) cycle++;   // One full A->B->A = one cycle
}
```

### See also

- [Fatigue Test Documentation](../examples/esp32/docs/fatigue_test.md) -- Full ESP-NOW fatigue test guide
- [Sensorless Homing Guide](special_features_sensorless_homing.md) -- StallGuard2 homing configuration
- [Source Code](../examples/esp32/main/fatigue_test_espnow/main.cpp) -- Production fatigue test implementation

---

## Running the Examples

### ESP32 (using build scripts)

```bash
cd examples/esp32
# Build a specific application (see app_config.yml for available apps)
./scripts/build_app.sh fatigue_test_espnow_unit
# Flash and monitor
./scripts/flash_app.sh /dev/ttyACM0
```

See the [ESP32 Examples README](../examples/esp32/README.md) for full setup instructions.

### Other Platforms

Compile with C++17 support and include the driver headers:
```bash
g++ -std=c++17 -I path/to/hf-tmc5160-driver/ your_app.cpp -o your_app
```

---

## Next Steps

| Where to go | What you'll find |
|-------------|------------------|
| [API Reference](api_reference.md) | Complete method signatures, return types, and parameter descriptions |
| [Platform Integration](platform_integration.md) | How to implement `MySPI` / `MyUART` for your board |
| [Configuration](configuration.md) | `DriverConfig` fields, chopper tuning, StealthChop parameters |
| [ESP32 example source](../examples/esp32/main/) | Production-quality implementations you can build and flash |
| [Troubleshooting](troubleshooting.md) | Common errors and recovery strategies |

---

**Navigation**
[<- API Reference](api_reference.md) | [Next: Troubleshooting ->](troubleshooting.md) | [Back to Index](index.md)
