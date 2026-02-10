---
layout: default
title: "Dev Board Setup"
description: "Hardware configuration and pinout for the TMC5160 development board"
nav_order: 1
parent: "ESP32 Examples"
permalink: /docs/esp32-examples/dev-board-setup/
---

# Development Board Setup

This document details the specific hardware configuration used for the TMC5160 development board (Trinamic Dev/Eval Kit) referenced in the examples. All tests and examples in this repository are configured by default to match this setup.

## 📝 Hardware Specifications

| Component | Model / Value | Notes |
|-----------|---------------|-------|
| **Microcontroller** | ESP32-C6 / ESP32-S3 | Generic ESP32 dev board |
| **Driver Chip** | TMC5160 | External MOSFET Driver |
| **MOSFETs** | **BSC072N08NS5** | Low Gate Charge ($Q_g \approx 24nC$, $Q_{gd} \approx 6-8nC$) |
| **Motor** | **17HS4401S-PG518** | NEMA 17 Stepper with 5.18:1 Planetary Gearbox |
| **Sense Resistors** | **0.05 $\Omega$ / 1W** | Current Sensing Resistors (Changed from standard 0.075) |
| **Charge Pump Cap** | 22nF / **100V** | Connected between VCP and VS |
| **Gate Resistors** | **10 $\Omega$ 1%** | Per gate drive line (Stability) |
| **Power Supply** | Up to 60V DC | VSA connected to VM (Motor Supply) |
| **Logic Power** | 5V Input | 5VOUT -> VCC, 3.3V LDO -> VCC_IO |

## 🔌 Pin Configuration

The software is configured to use the following GPIO mapping by default (defined in `examples/esp32/main/test_config/esp32_tmc51x0_test_config.hpp`).

### SPI Interface
| Signal | ESP32 Pin | TMC5160 Pin | Description |
|--------|-----------|-------------|-------------|
| **MOSI** | GPIO 6    | SDI         | SPI Data Input |
| **MISO** | GPIO 2    | SDO         | SPI Data Output |
| **SCLK** | GPIO 5    | SCK         | SPI Clock |
| **CS**   | GPIO 18   | CSN         | Chip Select (Active Low) |

### Control & Diagnostics
| Signal | ESP32 Pin | TMC5160 Pin | Description |
|--------|-----------|-------------|-------------|
| **EN** | GPIO 11   | DRV_ENN     | Enable (Active Low to enable driver) |
| **CLK**| GPIO 10   | CLK16       | External Clock Input (Ground if using internal) |
| **DIAG0**| GPIO 23 | DIAG0       | Diagnostic Output 0 |
| **DIAG1**| GPIO 15 | DIAG1       | Diagnostic Output 1 |

### Hardware-Fixed Pins
These pins determine the communication mode and are **hardwired** on the board.

| Signal | Level | Description |
|--------|-------|-------------|
| **SPI_MODE** | **HIGH** | Selects SPI Mode |
| **SD_MODE**  | **LOW**  | Selects Internal Ramp Generator Mode |

---

## ⚙️ Critical Driver Settings

### Power Stage Configuration
*   **MOSFET Type**: Low Gate Charge ($Q_{gd} < 10nC$) with 10 $\Omega$ Gate Resistors.
*   **DRVSTRENGTH**: **0** (Weakest) - Safe for these MOSFETs.
*   **BBMTIME**: **0** (Minimum) - ~100ns dead time is sufficient.
*   **BBMCLKS**: **0**

### Motor Current Settings (Sense Resistor = 0.05 $\Omega$)
With the **0.05 $\Omega$** sense resistors, the full-scale current capability is higher than standard boards.
*   **Full Scale Current** ($V_{FS} = 325mV$): $I_{FS} = \frac{0.325V}{0.05\Omega} = 6.5A$
*   **Global Scaler**: **160** (Recommended range)
*   **Target Run Current**: ~1.0A RMS (Safe for 1.68A motor testing)
    *   $I_{coil} = \frac{325mV}{0.05\Omega} \times \frac{160}{256} \times \frac{IRUN+1}{32}$
    *   For 1.0A RMS (approx 1.41A Peak): `IRUN` $\approx$ 8-9.
*   **Default Config**: `IRUN=8` (~1.0A RMS), `IHOLD=4` (~0.5A RMS).

### Mode
*   **StealthChop**: Disabled (SpreadCycle forced) by default in many test suites to prevent startup calibration surges. The fatigue test unit (`fatigue_test_espnow_unit`) enables StealthChop after bounds finding for smooth motion.

---

## 🐛 Troubleshooting Common Issues

### `uv_cp=1` (Charge Pump Undervoltage)
If you see this error immediately upon enabling the driver:
1.  **Check Capacitor**: Ensure the 22nF/100V capacitor between VCP and VS is present.
2.  **Check Wiring**: A short in the motor phases can drag down the supply voltage.
3.  **Voltage Drop**: The "Massive Current" surge might be dragging the PSU down. Ensure the PSU is stable at 24V+.

### Massive Current Draw at Startup
1.  **Wiring**: Verify Motor Phase resistance.
2.  **StealthChop**: Disable StealthChop (`en_pwm_mode=0`) to avoid the calibration current surge.

---

**Navigation:** [← Previous: Index](index.md) | [Index](index.md) | [Next: Motor Configuration →](motor_configuration.md)
