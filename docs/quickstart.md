---
layout: default
title: "Quick Start"
description: "Get up and running with the TMC51x0 driver (TMC5130 & TMC5160) in minutes"
nav_order: 2
parent: "Documentation"
permalink: /docs/quickstart/
---

# Quick Start

This guide will get you up and running with the TMC51x0 driver (TMC5130 & TMC5160) in just a few steps.

## Prerequisites

- [Driver installed](installation.md)
- [Hardware wired](hardware_setup.md)
- Communication interface implemented (see [Platform Integration](platform_integration.md))

## Understanding `Result<T>` -- Error Handling

Every driver method returns a `Result<T>` instead of throwing exceptions (which are disabled in most embedded toolchains). Here is all you need to know:

```cpp
// --- Check success ---
auto result = driver.rampControl.SetMaxSpeed(60.0f, tmc51x0::Unit::RPM);
if (!result) {                                       // boolean shorthand
    printf("Error: %s\n", result.ErrorMessage());    // "Communication interface error"
    tmc51x0::ErrorCode code = result.Error();        // ErrorCode::COMM_ERROR
    return;
}

// --- Get values ---
auto pos = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
if (pos.IsOk()) {
    float degrees = pos.Value();                     // safe access
}
float degrees = pos.ValueOr(0.0f);                   // safe default (no branch)

// --- Structured bindings (C++17) ---
auto [err, value] = driver.rampControl.GetCurrentSpeed(tmc51x0::Unit::RPM);
if (err == tmc51x0::ErrorCode::OK) { /* use value */ }
```

**All error codes** are in [`inc/tmc51x0_result.hpp`](../inc/tmc51x0_result.hpp) -- the most common are `COMM_ERROR`, `NOT_INITIALIZED`, `INVALID_VALUE`, and `TIMEOUT`. See the full table in the [README](../README.md#error-handling----the-resultt-pattern) or [Troubleshooting](troubleshooting.md).

> **Convention in this guide:** Simple examples omit some `Result` checks for readability.
> Production code should always check `Initialize()`, `Enable()`, and communication-dependent calls.

## Class Structure Overview

The TMC51x0 driver uses a **subsystem-based architecture** that organizes functionality into logical groups:

```cpp
tmc51x0::TMC51x0<MySPI> driver(spi);

// Organized subsystems for intuitive access
driver.rampControl      // Motion planning, positioning, velocity control
driver.motorControl     // Current control, chopper modes, stealthChop
driver.thresholds       // Velocity thresholds (TPWMTHRS/TCOOLTHRS/THIGH/VDCMIN)
driver.powerStage       // Power stage + protection (DRV_CONF/SHORT_CONF)
driver.io               // IOIN + mode pins (SPI_MODE/SD_MODE) + SDO_CFG0
driver.status           // Read-only monitoring (GSTAT/DRV_STATUS/RAMP_STAT/etc.)
driver.switches         // Reference switches/endstops (SW_MODE/XLATCH)
driver.events           // Motion events (X_COMPARE, RAMP_STAT clear)
driver.stallGuard       // StallGuard2 config + stop-on-stall/soft-stop
driver.encoder          // Encoder integration, closed-loop control
driver.tuning           // Automatic parameter optimization (SGT tuning)
driver.homing           // Sensorless and switch-based homing
driver.communication    // Multi-chip comm settings + raw register access helpers
driver.printer          // Debug register printing
```

Each subsystem provides focused methods for a specific aspect of motor control, making it easy to discover and use features. You don't need to learn all subsystems at once -- start with `rampControl` and `motorControl`, and explore others as needed.

## Minimal Example

Here's a complete working example:

```cpp
#include "inc/tmc51x0.hpp"

// 1. Implement your communication interface
//    You inherit from SpiCommInterface<YourClass> (CRTP -- Curiously Recurring Template Pattern).
//    The driver calls these methods to talk to the TMC chip.
//    See platform_integration.md for ESP32, STM32, and Arduino implementations.
class MySPI : public tmc51x0::SpiCommInterface<MySPI> {
public:
    CommMode GetMode() const noexcept { return CommMode::SPI; }

    Result<void> SpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) {
        // Pull CSN low, clock out tx[] while reading rx[], pull CSN high.
        // For daisy-chaining: CSN stays low for the entire multi-device transfer.
        return Result<void>();     // Return Result(ErrorCode::COMM_ERROR) on failure
    }
    Result<void> GpioSet(TMC51x0CtrlPin pin, GpioSignal signal) {
        // Drive the DRV_ENN (enable) or other control pins high/low.
        return Result<void>();
    }
    Result<GpioSignal> GpioRead(TMC51x0CtrlPin pin) {
        // Read a control pin state (e.g. DIAG0/DIAG1 for StallGuard events).
        GpioSignal signal = GpioSignal::INACTIVE;
        return Result<GpioSignal>(signal);
    }
    void DebugLog(int level, const char* tag, const char* format, va_list args) {
        // Optional: route driver debug messages to your logging framework.
    }
    void DelayMs(uint32_t ms) {
        // Platform-specific millisecond delay (used during init and tuning).
    }
    void DelayUs(uint32_t us) {
        // Platform-specific microsecond delay (used for SPI timing).
    }
};

int main() {
    // 2. Create and initialize your SPI hardware
    MySPI spi;
    spi.Initialize();

    // 3. Create driver -- template parameter links it to your SPI class at compile time
    tmc51x0::TMC51x0<MySPI> driver(spi);

    // 4. Initialize -- provide motor specs so the driver can calculate current registers
    tmc51x0::DriverConfig cfg{};
    cfg.motor_spec.rated_current_ma  = 2000;   // From motor datasheet
    cfg.motor_spec.sense_resistor_mohm = 50;   // From your PCB (e.g. 0.05 Ohm)
    cfg.motor_spec.supply_voltage_mv = 24000;  // Your power supply
    // IRUN, IHOLD, GLOBAL_SCALER are calculated automatically -- don't set them manually

    auto init_result = driver.Initialize(cfg);
    if (!init_result) {
        printf("Init failed: %s\n", init_result.ErrorMessage());
        return -1;                             // Common cause: SPI wiring or clock issue
    }

    // 5. Configure motion profile (RampControl subsystem)
    driver.rampControl.SetRampMode(tmc51x0::RampMode::POSITIONING);
    driver.rampControl.SetMaxSpeed(60.0f, tmc51x0::Unit::RPM);          // 60 RPM peak
    driver.rampControl.SetAcceleration(5.0f, tmc51x0::Unit::RevPerSec); // 5 rev/s^2 ramp up
    driver.rampControl.SetDeceleration(5.0f, tmc51x0::Unit::RevPerSec); // 5 rev/s^2 ramp down

    // 6. Enable motor output (pulls DRV_ENN active via your GpioSet)
    driver.motorControl.Enable();

    // 7. Command a move -- 180 degrees
    driver.rampControl.SetTargetPosition(180.0f, tmc51x0::Unit::Deg);

    // 8. Wait for the ramp generator to finish
    while (true) {
        auto reached = driver.rampControl.IsTargetReached();
        if (reached.IsOk() && reached.Value()) {
            printf("Target reached!\n");
            break;
        }
        if (reached.IsErr()) {
            printf("Error: %s\n", reached.ErrorMessage());
            break;
        }
        // Add a delay in your main loop to avoid busy-spinning
        // e.g. vTaskDelay(pdMS_TO_TICKS(10));  // FreeRTOS
    }

    return 0;
}
```

## What Just Happened -- Step by Step

| Line(s) | What the driver does internally |
|---------|---------------------------------|
| `Initialize(cfg)` | Resets the TMC chip via SPI, writes GCONF, calculates IRUN/IHOLD/GLOBAL_SCALER from your motor specs, configures chopper defaults and StealthChop PWM |
| `SetRampMode(POSITIONING)` | Writes RAMPMODE=0 -- the internal trapezoidal ramp generator will decelerate and stop at the target |
| `SetMaxSpeed(60, RPM)` | Converts 60 RPM to internal clock ticks and writes VMAX |
| `SetAcceleration(5, RevPerSec)` | Converts 5 rev/s^2 to internal units and writes AMAX |
| `Enable()` | Calls your `GpioSet()` to pull DRV_ENN low, energizing the motor coils |
| `SetTargetPosition(180, Deg)` | Converts 180 degrees to microsteps (using full_steps_per_rev x microstepping) and writes XTARGET |
| `IsTargetReached()` | Reads the `position_reached` flag from the RAMP_STAT register -- returns `Result<bool>` |

### Physical units

All position, speed, and acceleration methods accept a `Unit` parameter:

| Unit | Used for | Example |
|------|----------|---------|
| `Unit::Deg` | Position | `SetTargetPosition(90.0f, Unit::Deg)` |
| `Unit::Mm` | Position (requires `MechanicalSystem` in config) | `MoveRelative(10.0f, Unit::Mm)` |
| `Unit::RPM` | Speed | `SetMaxSpeed(60.0f, Unit::RPM)` |
| `Unit::RevPerSec` | Speed or acceleration (default) | `SetAcceleration(5.0f, Unit::RevPerSec)` |

If you omit the unit, `RevPerSec` is assumed for velocity/acceleration and microsteps for position.

## Exploring Subsystems

Once you have basic motion working, explore the other subsystems. You don't need all of these -- pick what your application requires.

### Status Monitoring

Check the driver's health at any time. Useful in your main loop or after errors.

```cpp
// Read StallGuard2 value: 0 = stalled, 1023 = no load (only valid in SpreadCycle above TCOOLTHRS)
auto sg = driver.stallGuard.GetStallGuardResult();
if (sg.IsOk()) {
    printf("Motor load (SG_RESULT): %u\n", sg.Value());
}

// Verify that register writes round-tripped correctly (useful during bring-up)
auto verify = driver.status.VerifySetup();
if (!verify) {
    printf("Setup mismatch: %s\n", verify.ErrorMessage());
}
```

**Deeper dive:** [API Reference -- Status Subsystem](api_reference.md)

### Automatic StallGuard Tuning

Instead of guessing the SGT threshold, let the driver sweep it for you.

```cpp
tmc51x0::StallGuardTuningResult result;
auto tune = driver.tuning.AutoTuneStallGuard(
    0.6f, result,                              // Target velocity: 0.6 rev/s (~36 RPM)
    0, 63,                                     // SGT search range: min, max
    0.06f,                                     // Acceleration: 0.06 rev/s^2
    0.18f, 0.9f,                               // Velocity range: 30%-150% of target
    tmc51x0::Unit::RevPerSec,                  // Velocity unit
    tmc51x0::Unit::RevPerSec,                  // Acceleration unit
    0.3f                                       // Current reduction: 30% (recommended)
);

if (tune.IsOk()) {
    printf("Optimal SGT: %d\n", result.optimal_sgt);
    driver.stallGuard.ConfigureStallGuard(result.optimal_sgt, true);
} else {
    printf("Tuning failed: %s\n", tune.ErrorMessage());
}
```

**Deeper dive:** [Advanced Configuration -- Automatic Tuning](special_features_advanced_configuration.md#automatic-tuning-with-comprehensive-velocity-range-analysis) |
[Source](../examples/esp32/main/tuning/stallguard_tuning.cpp)

### Bounds Finding and Homing

Find mechanical limits using StallGuard2 (sensorless) and home to the center.

```cpp
using Homing = tmc51x0::TMC51x0<MySPI>::Homing;

Homing::BoundsOptions opt{};
opt.speed_unit      = tmc51x0::Unit::RPM;
opt.position_unit   = tmc51x0::Unit::Deg;
opt.search_speed    = 30.0f;          // 30 RPM search speed
opt.search_span     = 360.0f;         // Max 360 degrees per direction
opt.backoff_distance = 5.0f;          // Back off 5 degrees after hitting a stop
opt.timeout_ms      = 10000;          // 10 s timeout per direction
opt.search_accel    = 5.0f;
opt.accel_unit      = tmc51x0::Unit::RevPerSec;

Homing::HomeConfig home{};
home.mode = Homing::HomePlacement::AtCenter;

auto bounds = driver.homing.FindBounds(Homing::BoundsMethod::StallGuard, opt, home);
if (bounds.IsOk()) {
    printf("Span: %.1f deg\n", bounds.Value().span);
} else {
    printf("Homing failed: %s\n", bounds.ErrorMessage());
}
```

**Deeper dive:** [Sensorless Homing Guide](special_features_sensorless_homing.md) |
[Source](../examples/esp32/main/fatigue_test_espnow/main.cpp)

### Encoder Integration

Add position feedback for step-loss detection with an ABN incremental encoder.

```cpp
tmc51x0::EncoderConfig enc_cfg{};
driver.encoder.Configure(enc_cfg);
driver.encoder.SetResolution(200, 4096, false);  // 200 steps/rev motor, 4096 PPR encoder
driver.encoder.SetAllowedDeviation(100);          // Flag deviation > 100 steps
```

**Deeper dive:** [Examples -- Encoder Closed-Loop Control](examples.md#example-4-encoder-closed-loop-control) |
[Source](../examples/esp32/main/sensors/abn_encoder_reader.cpp)

## Common First-Time Issues

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `COMM_ERROR` on `Initialize()` | SPI wiring or clock speed | Check MOSI/MISO/SCK/CSN connections; try lower clock (1 MHz) |
| Motor doesn't move after `Enable()` | DRV_ENN pin not wired or `GpioSet()` not implemented | Verify your `GpioSet` pulls the enable pin active |
| Motor vibrates but doesn't rotate | Current too low or wrong sense resistor value | Double-check `rated_current_ma` and `sense_resistor_mohm` against your hardware |
| `HARDWARE_ERROR` during motion | Supply voltage issue or thermal shutdown | Check VM voltage, motor temperature, add cooling if needed |
| StallGuard always reads 0 | StealthChop is active (SG only works in SpreadCycle) | Disable StealthChop or ensure speed is above TPWMTHRS |

See [Troubleshooting](troubleshooting.md) for a complete guide including error recovery strategies.

## Multi-Chip Daisy-Chaining

For multiple TMC51x0 drivers on a single SPI bus:

```cpp
// Create one SPI interface (shared by all chips)
MySPI spi;
spi.Initialize();

// Option 1: Use TMC51x0DaisyChain helper class (recommended)
tmc51x0::TMC51x0DaisyChain<MySPI, 5> chain(spi, 3);
// Auto-detects chain length and configures properly
auto chain_result = chain.InitializeAll(cfg);
if (!chain_result) {
    printf("Chain initialization error: %s\n", chain_result.ErrorMessage());
    return -1;
}

// Access individual drivers
auto& driver1 = chain[0]; // Position 0 (first chip)
auto& driver2 = chain[1]; // Position 1 (second chip)
auto& driver3 = chain[2]; // Position 2 (third chip)

// Option 2: Manual setup with individual driver instances
tmc51x0::TMC51x0<MySPI> driver1(spi);
tmc51x0::TMC51x0<MySPI> driver2(spi);
tmc51x0::TMC51x0<MySPI> driver3(spi);

// Set daisy chain position and length on each driver
driver1.communication.SetDaisyChainPosition(0);
driver2.communication.SetDaisyChainPosition(1);
driver3.communication.SetDaisyChainPosition(2);

// Initialize each driver
driver1.Initialize(cfg);
driver2.Initialize(cfg);
driver3.Initialize(cfg);
```

See [Multi-Chip Communication](special_features_multi_chip.md) for detailed information.

## Next Steps

| Where to go | What you'll find |
|-------------|------------------|
| [Examples](examples.md) | Progressively harder walkthroughs: velocity mode, StealthChop, encoder, StallGuard, fatigue testing |
| [API Reference](api_reference.md) | Complete method signatures, return types, and parameter descriptions for every subsystem |
| [Configuration](configuration.md) | All `DriverConfig` fields, chopper tuning, and StealthChop parameters |
| [Multi-Chip Communication](special_features_multi_chip.md) | SPI daisy-chain and UART multi-node setups |
| [Troubleshooting](troubleshooting.md) | Error codes, common failures, and recovery strategies |

---

**Navigation**
[<- Installation](installation.md) | [Next: Hardware Setup ->](hardware_setup.md) | [Back to Index](index.md)

