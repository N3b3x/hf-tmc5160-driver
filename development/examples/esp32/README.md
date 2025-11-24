# TMC5160 Driver - ESP32 Examples

This directory contains comprehensive examples for using the TMC5160 stepper motor driver with ESP32 microcontrollers.

## 🛠️ Hardware Setup

These examples are configured by default for a specific **Trinamic TMC5160 Dev/Eval Kit** setup.

**Quick Summary:**
*   **Driver:** TMC5160
*   **MOSFETs:** BSC072N08NS5 (Low Gate Charge)
*   **Motor:** 17HS4401S-PG518 (NEMA 17 with 5.18:1 Gearbox)
*   **Sense Resistors:** 0.05 $\Omega$
*   **Current Settings:** IRUN=8 (~0.8A RMS), DRVSTRENGTH=0

👉 **[Full Hardware Configuration & Pinout](docs/dev_board_setup.md)**  
*Please read the detailed setup guide above before running any examples to ensure your hardware matches the configuration and avoid damage.*

## 📂 Available Examples

| Example | Description |
|---------|-------------|
| `sinusoidal` | **Main Demo:** Smooth back-and-forth motion using internal ramp generator. |
| `core_comprehensive_test` | Basic register read/write and SPI communication verification. |
| `motor_control_comprehensive_test` | Tests current settings, chopper modes, and basic motion. |
| `ramp_control_comprehensive_test` | Validates velocity and positioning modes. |
| `diagnostics_comprehensive_test` | Reads status registers (GSTAT, DRV_STATUS) and checks flags. |
| `protection_comprehensive_test` | Tests short circuit and overtemperature protection thresholds. |

## 🚀 Getting Started

1.  **Setup Hardware:** Connect your ESP32 and TMC5160 according to the [Hardware Setup Guide](docs/dev_board_setup.md).
2.  **Configure:** Check `main/esp32_tmc5160_bus_config.hpp` if you need to change pin assignments.
3.  **Build & Flash:**
    ```bash
    idf.py set-target esp32s3  # Or esp32, esp32c3, etc.
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor
    ```

## ⚠️ Critical Warnings

*   **Charge Pump:** Ensure the **22nF/100V** capacitor is connected between VCP and VS. Missing this will cause `uv_cp=1` errors.
*   **Wiring:** Verify motor phases are not shorted before powering up.
*   **Current:** The default configuration is safe for the specified 0.05 Ohm sense resistors. If using standard 0.075 Ohm resistors, the current will be lower (safe).

