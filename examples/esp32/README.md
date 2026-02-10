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

| App | Description |
|-----|-------------|
| `internal_ramp_sinusoidal` | Back-and-forth motion using internal ramp positioning mode. |
| `internal_ramp_comprehensive_test` | Main test suite: core, motor, ramp, diagnostics, protection. |
| `fatigue_test_espnow_unit` | Point-to-point fatigue motion, ESP-NOW + UART, bounds finding. |
| `stallguard_tuning` | Automatic StallGuard2 (SGT) tuning for sensorless homing. |
| `bounds_finding_test` | Standalone bounds finding and homing test. |
| `spi_daisy_chain_comprehensive_test` | Multi-motor SPI daisy chain tests (2+ drivers required). |
| `uart_multi_node_comprehensive_test` | Multi-node UART tests (2+ drivers, placeholder impl). |
| `abn_encoder_reader` | ABN incremental encoder position reader. |
| `i2c_scan` | I2C bus scanner for OLED UI board. |

Run `./scripts/build_app.sh list` to see all apps and build types.

## 🚀 Getting Started

![What you need for the ESP32 build process](../assets/images/what-you-need.png)

1.  **Setup Hardware:** Connect your ESP32 and TMC5160 according to the [Hardware Setup Guide](docs/dev_board_setup.md).
2.  **Install ESP-IDF and project tools environment:**
    ```bash
    cd examples/esp32/scripts
    ./manage_idf.sh install                     # Install required ESP-IDF versions from app_config.yml
    source <(./manage_idf.sh export release/v5.5)
    ```
3.  **Explore available apps and build types:**
    ```bash
    cd ..                                       # Back to examples/esp32 project root
    ./scripts/build_app.sh list                 # Show apps, build types, and IDF versions
    ```
4.  **Build an app (example: fatigue_test_espnow_unit):**
    ```bash
    ./scripts/build_app.sh fatigue_test_espnow_unit Debug     # or Release
    ```
5.  **Flash and monitor using flash_app.sh:**
    ```bash
    ./scripts/flash_app.sh --port /dev/ttyACM0 flash_monitor fatigue_test_espnow_unit Debug
    # or, for a release build:
    ./scripts/flash_app.sh --port /dev/ttyACM0 flash_monitor fatigue_test_espnow_unit Release
    ```
6.  **More information about the build/flash system:**
    - See `scripts/README.md` for an overview of the ESP-IDF project tools.
    - Full documentation: [HardFOC ESP-IDF Project Tools Docs](https://n3b3x.github.io/hf-espidf-project-tools/)

## ⚠️ Critical Warnings

*   **Charge Pump:** Ensure the **22nF/100V** capacitor is connected between VCP and VS. Missing this will cause `uv_cp=1` errors.
*   **Wiring:** Verify motor phases are not shorted before powering up.
*   **Current:** The default configuration is safe for the specified 0.05 Ohm sense resistors. If using standard 0.075 Ohm resistors, the current will be lower (safe).

