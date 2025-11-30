/**
 * @file esp32_tmc51x0_test_config.hpp
 * @brief ESP32 GPIO pin configuration and compile-time configuration for TMC51x0 driver (TMC5130 & TMC51x0)
 *
 * This file defines compile-time configuration for TMC51x0 driver initialization:
 * - **BoardConfig**: Board-specific hardware parameters (sense resistor, supply voltage, MOSFETs)
 * - **MotorConfig**: Motor-specific configurations (physical specs, chopper, StealthChop)
 * - **PlatformConfig**: Platform/application-specific configuration (reference switches, encoder, mechanical system)
 *
 * ## Configuration Hierarchy
 *
 * 1. **BoardConfig**: Hardware parameters that stay the same for the same driver board
 *    - Sense resistor value (0.05Ω typical)
 *    - Supply voltage (24V typical)
 *    - Clock frequency (12 MHz)
 *    - MOSFET characteristics
 *    - Short protection defaults
 *
 * 2. **MotorConfig**: Motor-specific parameters that depend on the motor being used
 *    - Motor physical specs (steps/rev, rated current, resistance, inductance)
 *    - Chopper settings (TOFF, TBL, HSTRT, HEND)
 *    - StealthChop settings (PWM frequency, offset, autoscale)
 *    - Current settings (calculated automatically from motor specs)
 *
 * 3. **PlatformConfig**: Application/platform-specific parameters
 *    - Reference switches (endstops) configuration
 *    - Encoder configuration
 *    - Mechanical system (gearing, leadscrew, belt drive)
 *
 * ## Usage
 *
 * ### Unified Test Rig Selection (Recommended)
 *
 * ```cpp
 * // Select test rig at compile time - automatically selects motor, board, and platform
 * static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
 *     tmc51x0_test_config::TestRigType::TEST_RIG_CORE_DRIVER;
 *
 * // 1. Configure driver from test rig (automatically configures motor, board, and platform)
 * tmc51x0::DriverConfig cfg{};
 * tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
 *
 * // 2. Initialize driver
 * driver.Initialize(cfg);
 *
 * // 3. Configure platform-specific features after initialization
 * auto ref_cfg = tmc51x0_test_config::GetTestRigReferenceSwitchConfig<SELECTED_TEST_RIG>();
 * driver.rampControl.ConfigureReferenceSwitch(ref_cfg);
 *
 * auto enc_cfg = tmc51x0_test_config::GetTestRigEncoderConfig<SELECTED_TEST_RIG>();
 * driver.encoder.Configure(enc_cfg);
 * ```
 *
 * ### Manual Selection (Advanced)
 *
 * For advanced use cases, you can still manually select motor, board, and platform:
 *
 * ```cpp
 * static constexpr tmc51x0_test_config::MotorType SELECTED_MOTOR = 
 *     tmc51x0_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;
 * static constexpr tmc51x0_test_config::BoardType SELECTED_BOARD = 
 *     tmc51x0_test_config::BoardType::BOARD_TMC51x0_EVAL;
 * static constexpr tmc51x0_test_config::PlatformType SELECTED_PLATFORM = 
 *     tmc51x0_test_config::PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG;
 *
 * tmc51x0::DriverConfig cfg{};
 * tmc51x0_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
 * tmc51x0_test_config::ApplyBoardConfig<SELECTED_BOARD>(cfg);
 * tmc51x0_test_config::ApplyPlatformConfig<SELECTED_PLATFORM>(cfg);
 * driver.Initialize(cfg);
 * ```
 *
 * This file also provides GPIO pin assignments and a complete Esp32SpiPinConfig structure.
 *
 * ## Test Rig Configuration
 *
 * The project uses two test rigs, each with a specific motor, board, and platform configuration:
 *
 * 1. **Core Driver Test Rig** (`TEST_RIG_CORE_DRIVER`):
 *    - Motor: 17HS4401S motor with gearbox (default) or direct drive
 *    - Board: TMC51x0 Evaluation Kit (BOARD_TMC51x0_EVAL)
 *    - Platform: Core Driver Test Rig (PLATFORM_CORE_DRIVER_TEST_RIG)
 *    - Features: Reference switches, encoder (AS5047U), gearbox or direct drive
 *    - Used for: Most comprehensive tests, core driver functionality
 *    - Examples: internal_ramp_comprehensive_test, spi_daisy_chain_comprehensive_test, stallguard_tuning
 *
 * 2. **Fatigue Test Rig** (`TEST_RIG_FATIGUE`):
 *    - Motor: Applied Motion 5034-369 motor (MOTOR_APPLIED_MOTION_5034)
 *    - Board: TMC51x0 Evaluation Kit (BOARD_TMC51x0_EVAL)
 *    - Platform: Fatigue Test Rig (PLATFORM_FATIGUE_TEST_RIG)
 *    - Features: Reference switches, encoder (AS5047U), direct drive (NEMA 34)
 *    - Used for: Bounds finding and sinusoidal motion testing
 *    - Examples: fatigue_test_stallguard, fatigue_test_encoder, internal_ramp_sinusoidal
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#ifndef ESP32_TMC51x0_TEST_CONFIG_HPP
#define ESP32_TMC51X0_TEST_CONFIG_HPP

#include "driver/gpio.h"
#include "tmc51x0_comm_interface.hpp"

namespace tmc51x0_test_config {

// SPI bus pins
constexpr gpio_num_t SPI_SCK = GPIO_NUM_5;   ///< SPI clock pin
constexpr gpio_num_t SPI_MOSI = GPIO_NUM_6;  ///< SPI MOSI (master out, slave in)
constexpr gpio_num_t SPI_MISO = GPIO_NUM_2;  ///< SPI MISO (master in, slave out)
constexpr gpio_num_t SPI_CS = GPIO_NUM_18;   ///< SPI chip select pin

// TMC51x0 control pins
constexpr gpio_num_t DRV_EN = GPIO_NUM_11;   ///< Driver enable pin (DRV_ENN)
constexpr gpio_num_t CLK = GPIO_NUM_10;      ///< Clock input pin (CLK, pin 12)

// Mode configuration pins (if available as control pins)
constexpr gpio_num_t SPI_MODE_PIN = GPIO_NUM_0;  ///< SPI_MODE pin (pin 22) - GPIO0 if available as control pin
constexpr gpio_num_t SD_MODE_PIN = GPIO_NUM_1;   ///< SD_MODE pin (pin 21) - GPIO1 if available as control pin

// Diagnostic pins
constexpr gpio_num_t DIAG0 = GPIO_NUM_23;    ///< Diagnostic output 0 (DIAG0_SWN, pin 26)
constexpr gpio_num_t DIAG1 = GPIO_NUM_15;    ///< Diagnostic output 1 (DIAG1_SWP, pin 27)

// Optional pins (not used in all tests, set to -1 if not connected)
constexpr gpio_num_t DIR = static_cast<gpio_num_t>(-1);      ///< Direction pin (optional)
constexpr gpio_num_t STEP = static_cast<gpio_num_t>(-1);     ///< Step pin (optional)

// SPI configuration
constexpr uint32_t SPI_CLOCK_SPEED_HZ = 500000;  ///< SPI clock speed 
constexpr spi_host_device_t SPI_HOST = SPI2_HOST; ///< SPI host device

/**
 * @brief Complete pin configuration for all tests
 *
 * This structure includes both SPI pins and TMC51x0 control pins,
 * allowing all GPIO assignments to be managed in one place.
 * Use this with the Esp32SPI constructor that takes Esp32SpiPinConfig.
 */
inline tmc51x0::Esp32SpiPinConfig GetDefaultPinConfig() noexcept {
  tmc51x0::Esp32SpiPinConfig config{};
  
  // SPI pins
  config.spi_mosi = static_cast<int>(SPI_MOSI);
  config.spi_miso = static_cast<int>(SPI_MISO);
  config.spi_sclk = static_cast<int>(SPI_SCK);
  config.spi_cs = static_cast<int>(SPI_CS);
  
  // TMC51x0 control pins
  config.tmc51x0_pins.en_pin = static_cast<int>(DRV_EN);
  config.tmc51x0_pins.clk_pin = static_cast<int>(CLK);
  config.tmc51x0_pins.diag0_pin = static_cast<int>(DIAG0);
  config.tmc51x0_pins.diag1_pin = static_cast<int>(DIAG1);
  config.tmc51x0_pins.dir_pin = static_cast<int>(DIR);
  config.tmc51x0_pins.step_pin = static_cast<int>(STEP);
  
  // Mode configuration pins (if available as control pins)
  config.tmc51x0_pins.spi_mode_pin = static_cast<int>(SPI_MODE_PIN);
  config.tmc51x0_pins.sd_mode_pin = static_cast<int>(SD_MODE_PIN);
  
  return config;
}

/**
 * @brief Motor type enumeration for compile-time motor selection
 * 
 * This enumeration defines the available motor configurations that can be selected
 * at compile time. Each motor has its own configuration namespace with optimized
 * settings for current, chopper, and StealthChop parameters.
 * 
 * MOTOR SELECTION GUIDE:
 * 
 * 1. MOTOR_17HS4401S_GEARBOX (default):
 *    - Model: 17HS4401S-PG518 with 5.18:1 planetary gearbox
 *    - Rated Current: 1.68A RMS per phase
 *    - Step Angle: 1.8° (200 steps/rev motor, ~1036 steps/rev output)
 *    - Holding Torque: 40Ncm (motor), ~207Ncm (output with gearbox)
 *    - Current Settings: IRUN=20 (~1.88A RMS), IHOLD=10 (~0.94A RMS)
 *    - Use for: Applications requiring high torque and precision positioning
 *    - Power Supply: 2-3A recommended
 * 
 * 2. MOTOR_17HS4401S_DIRECT:
 *    - Model: 17HS4401S direct drive (no gearbox)
 *    - Rated Current: 1.68A RMS per phase
 *    - Step Angle: 1.8° (200 steps/rev)
 *    - Holding Torque: 40Ncm
 *    - Current Settings: IRUN=20 (~1.88A RMS), IHOLD=10 (~0.94A RMS)
 *    - Use for: Applications requiring higher speed and lower torque
 *    - Power Supply: 2-3A recommended
 * 
 * 3. MOTOR_APPLIED_MOTION_5034:
 *    - Model: Applied Motion Products 5034-369 NEMA 34
 *    - Rated Current: 4.17A RMS per phase (bipolar series)
 *    - Step Angle: 1.8° (200 steps/rev)
 *    - Holding Torque: 636 oz-in (4.5 Nm)
 *    - Resistance: 0.84 Ohms/phase (bipolar series)
 *    - Inductance: 10.4 mH/phase (bipolar series)
 *    - Current Settings: IRUN=28 (~4.17A RMS), IHOLD=14 (~2.15A RMS)
 *    - Use for: High-torque applications requiring 4A+ current
 *    - WARNING: Requires power supply capable of 5A+ continuous current
 * 
 * USAGE:
 * In your example file, declare a static constexpr variable at global scope:
 * 
 *     static constexpr tmc51x0_test_config::MotorType SELECTED_MOTOR = 
 *         tmc51x0_test_config::MotorType::MOTOR_17HS4401S_GEARBOX;
 * 
 * Then use conditional compilation or if constexpr to select the motor namespace:
 * 
 *     if constexpr (SELECTED_MOTOR == MotorType::MOTOR_17HS4401S_GEARBOX) {
 *         namespace Motor = tmc51x0_test_config::MotorConfig_17HS4401S;
 *     } else if constexpr (SELECTED_MOTOR == MotorType::MOTOR_17HS4401S_DIRECT) {
 *         namespace Motor = tmc51x0_test_config::MotorConfig_17HS4401S_Direct;
 *     } else if constexpr (SELECTED_MOTOR == MotorType::MOTOR_APPLIED_MOTION_5034) {
 *         namespace Motor = tmc51x0_test_config::MotorConfig_AppliedMotion_5034_369;
 *     }
 */
/**
 * @brief Motor type enumeration
 * 
 * Selects which motor configuration to use at compile time.
 */
enum class MotorType {
    MOTOR_17HS4401S_GEARBOX,      ///< 17HS4401S with 5.18:1 planetary gearbox
    MOTOR_17HS4401S_DIRECT,       ///< 17HS4401S direct drive (no gearbox)
    MOTOR_APPLIED_MOTION_5034     ///< Applied Motion 5034-369 NEMA 34 (high torque)
};

/**
 * @brief Board type enumeration
 * 
 * Selects which board configuration to use at compile time.
 * Board configuration includes hardware parameters like sense resistor,
 * supply voltage, MOSFET characteristics, etc.
 */
enum class BoardType {
    BOARD_TMC51x0_EVAL,           ///< TMC51x0 Evaluation Kit (0.05Ω sense, 24V, BSC072N08NS5 MOSFETs)
    BOARD_TMC51x0_BOB,            ///< TMC51x0 Break-Out Board (0.11Ω sense, 24V, typical MOSFETs)
};

/**
 * @brief Platform type enumeration
 * 
 * Selects which platform configuration to use at compile time.
 * Platform configuration includes reference switches, encoder,
 * mechanical system type, etc.
 * 
 * @note For test rigs, use TestRigType instead - it automatically selects the platform.
 * This enum is kept for backward compatibility and future non-test-rig platforms.
 */
enum class PlatformType {
    PLATFORM_CORE_DRIVER_TEST_RIG,  ///< Core driver test rig (gearbox motor, reference switches, encoder)
    PLATFORM_FATIGUE_TEST_RIG,      ///< Fatigue test rig (Applied Motion motor, reference switches, encoder)
    // Add more platform types here as needed:
    // PLATFORM_3D_PRINTER,       ///< 3D printer platform configuration
    // PLATFORM_CNC_ROUTER,       ///< CNC router platform configuration
};

/**
 * @brief Test rig type enumeration
 * 
 * Selects which complete test rig configuration to use at compile time.
 * Each test rig automatically selects the appropriate motor, board, and platform.
 * This simplifies configuration by grouping related hardware together.
 * 
 * TEST RIG CONFIGURATIONS:
 * 
 * 1. TEST_RIG_CORE_DRIVER:
 *    - Motor: 17HS4401S with gearbox (MOTOR_17HS4401S_GEARBOX) or direct drive (MOTOR_17HS4401S_DIRECT)
 *    - Board: TMC51x0 Evaluation Kit (BOARD_TMC51x0_EVAL)
 *    - Platform: Core Driver Test Rig (PLATFORM_CORE_DRIVER_TEST_RIG)
 *    - Features: Reference switches, encoder (AS5047U), gearbox or direct drive
 *    - Used for: Comprehensive tests, core driver functionality
 * 
 * 2. TEST_RIG_FATIGUE:
 *    - Motor: Applied Motion 5034-369 (MOTOR_APPLIED_MOTION_5034)
 *    - Board: TMC51x0 Evaluation Kit (BOARD_TMC51x0_EVAL)
 *    - Platform: Fatigue Test Rig (PLATFORM_FATIGUE_TEST_RIG)
 *    - Features: Reference switches, encoder (AS5047U), direct drive (NEMA 34)
 *    - Used for: Bounds finding, sinusoidal motion, fatigue testing
 */
enum class TestRigType {
    TEST_RIG_CORE_DRIVER,  ///< Core driver test rig (17HS4401S motor, TMC51x0 EVAL board, reference switches, encoder)
    TEST_RIG_FATIGUE,      ///< Fatigue test rig (Applied Motion 5034-369 motor, TMC51x0 EVAL board, reference switches, encoder)
};

/**
 * @brief Motor Configuration for 17HS4401S-PG518 NEMA 17 Stepper Motor (WITH GEARBOX)
 * 
 * Model: 17HS4401S-PG518 (with Planetary Gearbox)
 * - Rated Current: 1.68A / Phase
 * - Step Angle (Motor): 1.8°
 * - Holding Torque (Motor): 40Ncm (before gearbox)
 * - Gear Ratio: 5.18:1 (Planetary)
 * - Steps/Rev (Output Shaft): 200 * 5.18 = 1036 steps
 * 
 * Driver Settings for Smoothness:
 * - Microsteps: 256 (MRES=0) for maximum smoothness
 * - Current: Run=1.88A (~112%), Hold=0.94A (~56%)
 * - Global Scaler: 160 (Optimal range >128)
 * - Chopper: TOFF=5, HEND=3, HSTRT=4 (Typical for NEMA17)
 */
namespace MotorConfig_17HS4401S {
    // Physical Motor Specs
    constexpr uint16_t RATED_CURRENT_MA = 1700;  // 1.7A (17HS4401 motor specification)
    constexpr float RESISTANCE_OHMS = 1.5f;      // 1.5 Ω per phase (17HS4401 motor specification)
    constexpr float INDUCTANCE_MH = 3.2f;         // 3.2 mH per phase (17HS4401 motor specification)
    constexpr float GEAR_RATIO = 5.18f;
    constexpr uint16_t MOTOR_FULL_STEPS = 200;
    constexpr uint16_t OUTPUT_FULL_STEPS = static_cast<uint16_t>(MOTOR_FULL_STEPS * GEAR_RATIO); // ~1036
    constexpr uint32_t SUPPLY_VOLTAGE_MV = 24000; // 24V (motor-specific supply voltage)

    // Driver Configuration
    // NOTE: Board has 0.05 Ohm Sense Resistors (1W, low-inductance type required).
    // Per datasheet table: RSENSE=0.05Ω → Max RMS=4.7A, Max Peak=6.6A (at CS=31, GLOBAL_SCALER=256)
    // 
    // Current Calculation (datasheet formula):
    // I_RMS = (GLOBAL_SCALER/256) * ((IRUN+1)/32) * (VFS/RSENSE) * (1/√2)
    // Where VFS = 0.325V (typical full-scale voltage)
    // 
    // With GLOBAL_SCALER=160, RSENSE=0.05Ω:
    // Max Peak Current = 6.5A * (160/256) = 4.06A Peak (at IRUN=31)
    // 
    // For 17HS4401S motor (1.68A RMS rated):
    // IRUN=20: I_RMS = 0.625 * 0.65625 * 6.5 * 0.707 = ~1.88A RMS
    // IRUN=20: I_Peak = 4.06A * 0.65625 = ~2.66A Peak
    // 
    // Increased to IRUN=20 for better StealthChop calibration:
    // - IRUN ≥ 8 is minimum for StealthChop automatic tuning
    // - IRUN 16-31 recommended for best microstep performance
    // - Current is slightly above motor rating but acceptable for testing
    //
    // Target currents (Tuned for specific application requirements)
    // Rated is 1.68A, but we drive slightly harder (1.88A) for better StealthChop calibration
    constexpr uint16_t TARGET_RUN_CURRENT_MA = 1880; 
    constexpr uint16_t TARGET_HOLD_CURRENT_MA = 940;
    
    // Microstepping for Maximum Smoothness
    constexpr tmc51x0::MicrostepResolution MRES = tmc51x0::MicrostepResolution::MRES_256; // Highest Resolution
    constexpr bool INTERPOLATION = true;         // Interpolation (always on for smoothness)
    
    // Chopper Configuration (SpreadCycle default for NEMA 17)
    constexpr uint8_t TOFF = 5;
    constexpr uint8_t HEND = 3;
    constexpr uint8_t HSTRT = 4;
    constexpr uint8_t TBL = 2;                   // Blank time 36 clocks
    
    // StealthChop Configuration
    constexpr bool STEALTH_AUTOSCALE = true;
    constexpr bool STEALTH_AUTOGRAD = true;
    constexpr uint8_t STEALTH_FREQ = 1;          // 1 = ~35kHz @ 12MHz clock (Good balance)
    constexpr uint8_t STEALTH_OFS = 30;

    // Power Stage Configuration (BSC072N08NS5)
    // Critical for preventing uv_cp errors with low Qg MOSFETs
    // BSC072N08NS5 has Qg(tot) ~6nC, Qgd (Miller) ~2nC - use <10nC category
    constexpr float MOSFET_MILLER_CHARGE_NC = 6.0f;  // Miller charge in nC (<10nC for BSC072N08NS5)
    constexpr uint32_t BBM_TIME_NS = 100;            // Break-before-make time in nanoseconds (~100ns for fast MOSFETs)

    // Default Ramp Profile (Tuned for NEMA 17 with gearbox)
    constexpr float RAMP_VSTART = 1.0f;
    constexpr float RAMP_VSTOP = 10.0f;
    constexpr float RAMP_VMAX = 10000.0f;  // Higher max speed for NEMA 17
    constexpr float RAMP_AMAX = 1000.0f;   // Higher acceleration
    constexpr float RAMP_DMAX = 1000.0f;
    constexpr float RAMP_A1 = 500.0f;
    constexpr float RAMP_D1 = 500.0f;
    constexpr float RAMP_V1 = 0.0f;
    constexpr float RAMP_TPOWERDOWN_MS = 100.0f;
    constexpr float RAMP_TZEROWAIT_MS = 0.0f;
    
    // Motor power down delay (IHOLDDELAY)
    // Total delay time for smooth motor power down after standstill
    // Typical range: 200-500ms for smooth transition without jerk
    constexpr float IHOLDDELAY_MS = 300.0f;  // 300ms total delay for smooth power down
    
    // StealthChop velocity threshold (TPWMTHRS)
    // Velocity below which StealthChop is active, above which SpreadCycle is used
    // Set to 0 to disable StealthChop (always use SpreadCycle)
    // Typical range: 500-1000 steps/s for NEMA 17 motors
    constexpr float STEALTH_VELOCITY_THRESHOLD = 800.0f;  // 800 steps/s threshold
}

/**
 * @brief Motor Configuration for 17HS4401S NEMA 17 Stepper Motor (DIRECT DRIVE, NO GEARBOX)
 * 
 * Model: 17HS4401S (Direct Drive, No Gearbox)
 * - Rated Current: 1.68A / Phase
 * - Step Angle: 1.8°
 * - Holding Torque: 40Ncm
 * - Gear Ratio: 1.0:1 (Direct Drive)
 * - Steps/Rev (Output Shaft): 200 steps
 * 
 * Driver Settings:
 * - Microsteps: 256 (MRES=0) for maximum smoothness
 * - Current: Run=1.88A (~112%), Hold=0.94A (~56%)
 * - Global Scaler: 160
 * - Chopper: TOFF=5, HEND=3, HSTRT=4 (Typical for NEMA17)
 */
namespace MotorConfig_17HS4401S_Direct {
    // Physical Motor Specs
    constexpr uint16_t RATED_CURRENT_MA = 1700;  // 1.7A (17HS4401 motor specification)
    constexpr float RESISTANCE_OHMS = 1.5f;      // 1.5 Ω per phase (17HS4401 motor specification)
    constexpr float INDUCTANCE_MH = 3.2f;         // 3.2 mH per phase (17HS4401 motor specification)
    constexpr float GEAR_RATIO = 1.0f;            // Direct drive (no gearbox)
    constexpr uint16_t MOTOR_FULL_STEPS = 200;
    constexpr uint16_t OUTPUT_FULL_STEPS = MOTOR_FULL_STEPS; // Same as motor (no gearbox)
    constexpr uint32_t SUPPLY_VOLTAGE_MV = 24000; // 24V (motor-specific supply voltage)

    // Driver Configuration
    // NOTE: Board has 0.05 Ohm Sense Resistors (1W, low-inductance type required).
    // Same current settings as geared version since motor is identical
    
    // Target currents (Same as geared version)
    constexpr uint16_t TARGET_RUN_CURRENT_MA = 1880; 
    constexpr uint16_t TARGET_HOLD_CURRENT_MA = 940;
    
    // Microstepping for Maximum Smoothness
    constexpr tmc51x0::MicrostepResolution MRES = tmc51x0::MicrostepResolution::MRES_256; // Highest Resolution
    constexpr bool INTERPOLATION = true;         // Interpolation (always on for smoothness)
    
    // Chopper Configuration (SpreadCycle default for NEMA 17)
    constexpr uint8_t TOFF = 5;
    constexpr uint8_t HEND = 3;
    constexpr uint8_t HSTRT = 4;
    constexpr uint8_t TBL = 2;                   // Blank time 36 clocks
    
    // StealthChop Configuration
    constexpr bool STEALTH_AUTOSCALE = true;
    constexpr bool STEALTH_AUTOGRAD = true;
    constexpr uint8_t STEALTH_FREQ = 1;          // 1 = ~35kHz @ 12MHz clock (Good balance)
    constexpr uint8_t STEALTH_OFS = 30;

    // Power Stage Configuration (BSC072N08NS5)
    // BSC072N08NS5 has Qg(tot) ~6nC, Qgd (Miller) ~2nC - use <10nC category
    constexpr float MOSFET_MILLER_CHARGE_NC = 6.0f;  // Miller charge in nC (<10nC for BSC072N08NS5)
    constexpr uint32_t BBM_TIME_NS = 100;            // Break-before-make time in nanoseconds (~100ns for fast MOSFETs)

    // Default Ramp Profile (Tuned for NEMA 17 direct drive)
    constexpr float RAMP_VSTART = 1.0f;
    constexpr float RAMP_VSTOP = 10.0f;
    constexpr float RAMP_VMAX = 10000.0f;  // Higher max speed for NEMA 17
    constexpr float RAMP_AMAX = 1000.0f;   // Higher acceleration
    constexpr float RAMP_DMAX = 1000.0f;
    constexpr float RAMP_A1 = 500.0f;
    constexpr float RAMP_D1 = 500.0f;
    constexpr float RAMP_V1 = 0.0f;
    constexpr float RAMP_TPOWERDOWN_MS = 100.0f;
    constexpr float RAMP_TZEROWAIT_MS = 0.0f;
    
    // Motor power down delay (IHOLDDELAY)
    // Total delay time for smooth motor power down after standstill
    // Typical range: 200-500ms for smooth transition without jerk
    constexpr float IHOLDDELAY_MS = 300.0f;  // 300ms total delay for smooth power down
    
    // StealthChop velocity threshold (TPWMTHRS)
    // Velocity below which StealthChop is active, above which SpreadCycle is used
    // Set to 0 to disable StealthChop (always use SpreadCycle)
    // Typical range: 500-1000 steps/s for NEMA 17 motors
    constexpr float STEALTH_VELOCITY_THRESHOLD = 800.0f;  // 800 steps/s threshold
}

/**
 * @brief Motor Configuration for Applied Motion 5034-369 NEMA 34 Stepper Motor
 * 
 * Model: Applied Motion Products 5034-369
 * - Rated Current (Bipolar Series): 4.17A / Phase
 * - Step Angle: 1.8°
 * - Holding Torque (Bipolar Series): 636 oz-in (4.5 Nm)
 * - Resistance (Bipolar Series): 0.84 Ohms/phase
 * - Inductance (Bipolar Series): 10.4 mH/phase
 * - Gear Ratio: 1.0:1 (Direct Drive)
 * - Steps/Rev (Output Shaft): 200 steps
 * 
 * Driver Settings:
 * - Microsteps: 256 (MRES=0) for maximum smoothness
 * - Current: Run=4.17A (100% rated), Hold=2.08A (50% rated)
 * - Global Scaler: 256 (Full scale for maximum current capacity)
 * - Chopper: TOFF=5, HEND=3, HSTRT=4 (Typical for NEMA34)
 * 
 * NOTE: This motor requires significantly higher current than NEMA 17 motors.
 * Ensure power supply can deliver adequate current (5A+ recommended).
 */
namespace MotorConfig_AppliedMotion_5034_369 {
    // Physical Motor Specs
    constexpr uint16_t RATED_CURRENT_MA = 4170;  // 4.17A RMS (bipolar series)
    constexpr float GEAR_RATIO = 1.0f;            // Direct drive (no gearbox)
    constexpr uint16_t MOTOR_FULL_STEPS = 200;
    constexpr uint16_t OUTPUT_FULL_STEPS = MOTOR_FULL_STEPS; // Same as motor (no gearbox)
    constexpr float RESISTANCE_OHMS = 0.84f;      // Bipolar series resistance
    constexpr float INDUCTANCE_MH = 10.4f;        // Bipolar series inductance
    constexpr uint32_t SUPPLY_VOLTAGE_MV = 24000; // 24V (motor-specific supply voltage)

    // Driver Configuration
    // NOTE: Board has 0.05 Ohm Sense Resistors (1W, low-inductance type required).
    // 
    // Current Calculation (datasheet formula):
    // I_RMS = (GLOBAL_SCALER/256) * ((IRUN+1)/32) * (VFS/RSENSE) * (1/√2)
    // Where VFS = 0.325V (typical full-scale voltage), RSENSE = 0.05Ω
    // 
    // With GLOBAL_SCALER=256 (full scale), RSENSE=0.05Ω:
    // Max Peak Current = 6.5A Peak (at IRUN=31)
    // Max RMS Current = 6.5A * 0.707 = 4.6A RMS (at IRUN=31)
    // 
    // For Applied Motion 5034-369 motor (4.17A RMS rated):
    // Target: 4.17A RMS = 5.9A Peak
    // But max available is 4.6A RMS, so we'll use maximum available:
    // IRUN=31: I_RMS = 1.0 * 1.0 * 6.5 * 0.707 = ~4.6A RMS (slightly above rated)
    // IRUN=28: I_RMS = 1.0 * 0.90625 * 6.5 * 0.707 = ~4.17A RMS (exact match)
    // 
    // Using IRUN=28 for exact rated current, or IRUN=31 for maximum available
    // Target currents
    // Rated: 4.17A. Board max with 0.05R is ~4.6A.
    // We use rated current.
    constexpr uint16_t TARGET_RUN_CURRENT_MA = 4170; // 100% rated
    constexpr uint16_t TARGET_HOLD_CURRENT_MA = 2150; // ~50%
    
    // Microstepping for Maximum Smoothness
    constexpr tmc51x0::MicrostepResolution MRES = tmc51x0::MicrostepResolution::MRES_256; // Highest Resolution
    constexpr bool INTERPOLATION = true;         // Interpolation (always on for smoothness)
    
    // Chopper Configuration (SpreadCycle for NEMA 34)
    // NEMA 34 motors typically need slightly different chopper settings
    constexpr uint8_t TOFF = 5;
    constexpr uint8_t HEND = 3;
    constexpr uint8_t HSTRT = 4;
    constexpr uint8_t TBL = 2;                   // Blank time 36 clocks
    
    // StealthChop Configuration
    // Higher current motors may need different StealthChop settings
    constexpr bool STEALTH_AUTOSCALE = true;
    constexpr bool STEALTH_AUTOGRAD = true;
    constexpr uint8_t STEALTH_FREQ = 1;          // 1 = ~35kHz @ 12MHz clock
    constexpr uint8_t STEALTH_OFS = 30;          // May need adjustment for higher current motor

    // Power Stage Configuration (BSC072N08NS5)
    // Higher current may require different drive strength
    constexpr uint8_t DRV_STRENGTH = 0;          // Weakest setting for low Qg MOSFETs (<10nC)
    constexpr uint8_t BBM_TIME = 0;              // 0 = ~100ns (Sufficient for fast MOSFETs)
    constexpr uint8_t BBM_CLKS = 0;              // 0 = Off

    // Default Ramp Profile (Tuned for NEMA 34)
    // Lower acceleration due to higher rotor inertia
    constexpr float RAMP_VSTART = 1.0f;
    constexpr float RAMP_VSTOP = 10.0f;
    constexpr float RAMP_VMAX = 5000.0f;  // Lower max speed for NEMA 34
    constexpr float RAMP_AMAX = 500.0f;   // Lower acceleration
    constexpr float RAMP_DMAX = 500.0f;
    constexpr float RAMP_A1 = 250.0f;
    constexpr float RAMP_D1 = 250.0f;
    constexpr float RAMP_V1 = 0.0f;
    constexpr float RAMP_TPOWERDOWN_MS = 100.0f;
    constexpr float RAMP_TZEROWAIT_MS = 0.0f;
    
    // Motor power down delay (IHOLDDELAY)
    // Total delay time for smooth motor power down after standstill
    // Higher values for high-torque motors to prevent mechanical shock
    // Typical range: 300-500ms for NEMA 34 motors
    constexpr float IHOLDDELAY_MS = 400.0f;  // 400ms total delay for smooth power down
    
    // StealthChop velocity threshold (TPWMTHRS)
    // Velocity below which StealthChop is active, above which SpreadCycle is used
    // Set to 0 to disable StealthChop (always use SpreadCycle)
    // Typical range: 500-1000 steps/s for NEMA 34 motors
    constexpr float STEALTH_VELOCITY_THRESHOLD = 600.0f;  // 600 steps/s threshold (lower for larger motor)
}

/**
 * @brief Test Rig Configuration Defaults
 * 
 * Contains tuned parameters for various test scenarios.
 * These are "best guess" defaults for a typical setup with the 17HS4401S motor.
 * 
 * @note Reference switches and encoder configuration have been moved to PlatformConfig_TestRig.
 * Use PlatformConfig_TestRig::ReferenceSwitches and PlatformConfig_TestRig::Encoder instead.
 */
namespace TestConfig_17HS4401S {
    
    // --- Sensorless Homing / StallGuard Configuration ---
    namespace StallGuard {
        // SGT: StallGuard2 Threshold (-64 to +63).
        // Lower values = Higher sensitivity (stops easier).
        // Higher values = Lower sensitivity (needs more force to stop).
        // Updated to SGT=10 based on automatic tuning results (stallguard_tuning example)
        // This value was found to work well at 30-40k steps/s for the 17HS4401S motor
        constexpr int8_t SGT_HOMING = 10;  
        
        // CoolStep configuration for homing (usually disabled or tuned for stability)
        constexpr bool FILTER_ENABLED = true;
        constexpr uint8_t SEMIN = 2;
        constexpr uint8_t SEMAX = 5;
    }

    // --- Motion Profiles ---
    namespace Motion {
        // Homing Speeds (Steps/s)
        // Needs to be fast enough for back-EMF sensing (Sensorless)
        constexpr float HOMING_SEARCH_SPEED = 20000.0f; // ~30-40 RPM output (at 256 usteps, 5.18 gear)
        constexpr float HOMING_SWITCH_SPEED = 2000.0f;  // Slower for precision
        
        // Bounds Finding Test
        constexpr float BOUNDS_SEARCH_SPEED = 20000.0f;
        constexpr uint32_t HOMING_TIMEOUT_MS = 30000;
        
        // Fatigue Test Defaults
        constexpr float FATIGUE_FREQ_HZ = 0.5f;
        constexpr float FATIGUE_AMPLITUDE_DEG = 60.0f;
        constexpr uint32_t DWELL_MS = 2000;
    }
}

/**
 * @brief Board hardware configuration for TMC51x0 Evaluation Kit
 * 
 * Defines board-specific hardware parameters for the TMC51x0 Evaluation Kit.
 * These values are used for automatic current calculation and power stage configuration.
 * 
 * **Board-specific**: These parameters depend on the TMC51x0 EVAL kit hardware:
 * - Sense resistor value (0.05Ω / 50 mOhm)
 * - Clock frequency (12 MHz internal clock)
 * - MOSFET characteristics (BSC072N08NS5: Miller charge ~6nC, BBM time ~100ns)
 * - Short protection defaults
 * 
 * **Note**: Supply voltage is motor-specific and should be set from MotorConfig, not BoardConfig.
 * The board can handle 12-36V, but the actual voltage depends on the motor being driven.
 * 
 * These values stay the same regardless of which motor is connected or what platform it's used on.
 */
namespace BoardConfig_TMC51x0_EVAL {
    // Driver board hardware configuration
    constexpr uint32_t SENSE_RESISTOR_MOHM = 50;      ///< Sense resistor value in milliohms (0.05Ω)
    constexpr uint32_t CLOCK_FREQUENCY_HZ = 0; ///< TMC51x0 clock frequency in Hz (0 = use internal 12 MHz oscillator, CLK pin tied to GND)
    
    // Power stage MOSFET characteristics (BSC072N08NS5)
    constexpr float MOSFET_MILLER_CHARGE_NC = 6.0f;   ///< MOSFET Miller charge in nC (<10nC for BSC072N08NS5)
    constexpr uint32_t BBM_TIME_NS = 100;             ///< Break-before-make time in nanoseconds (~100ns for fast MOSFETs)
    
    // Short protection defaults (can be overridden per motor if needed)
    constexpr uint16_t S2VS_VOLTAGE_MV = 625;         ///< Short to VS voltage threshold in mV (0 = auto = 625mV)
    constexpr uint16_t S2G_VOLTAGE_MV = 625;          ///< Short to GND voltage threshold in mV (0 = auto = 625mV)
}

/**
 * @brief Board hardware configuration for TMC51x0 Break-Out Board (BOB)
 * 
 * Defines board-specific hardware parameters for the TMC51x0 Break-Out Board.
 * These values are used for automatic current calculation and power stage configuration.
 * 
 * **Board-specific**: These parameters depend on the TMC51x0 BOB hardware:
 * - Sense resistor value (0.11Ω / 110 mOhm - typical for BOB)
 * - Clock frequency (12 MHz internal clock)
 * - MOSFET characteristics (typical MOSFETs: Miller charge ~30nC, BBM time ~200ns)
 * - Short protection defaults
 * 
 * **Note**: BOB boards may vary. Adjust these values based on your specific BOB hardware.
 * Common BOB configurations use 0.11Ω sense resistors and standard MOSFETs.
 * 
 * **Note**: Supply voltage is motor-specific and should be set from MotorConfig, not BoardConfig.
 * The board can handle 12-36V, but the actual voltage depends on the motor being driven.
 * 
 * These values stay the same regardless of which motor is connected or what platform it's used on.
 */
namespace BoardConfig_TMC51x0_BOB {
    // Driver board hardware configuration
    constexpr uint32_t SENSE_RESISTOR_MOHM = 110;     ///< Sense resistor value in milliohms (0.11Ω - typical for BOB)
    constexpr uint32_t CLOCK_FREQUENCY_HZ = 0; ///< TMC51x0 clock frequency in Hz (0 = use internal 12 MHz oscillator, CLK pin tied to GND)
    
    // Power stage MOSFET characteristics (typical BOB MOSFETs)
    constexpr float MOSFET_MILLER_CHARGE_NC = 30.0f;  ///< MOSFET Miller charge in nC (~30nC for typical BOB MOSFETs)
    constexpr uint32_t BBM_TIME_NS = 200;             ///< Break-before-make time in nanoseconds (~200ns for typical MOSFETs)
    
    // Short protection defaults (can be overridden per motor if needed)
    constexpr uint16_t S2VS_VOLTAGE_MV = 625;         ///< Short to VS voltage threshold in mV (0 = auto = 625mV)
    constexpr uint16_t S2G_VOLTAGE_MV = 625;          ///< Short to GND voltage threshold in mV (0 = auto = 625mV)
}


/**
 * @brief Platform configuration for Core Driver Test Rig
 * 
 * Defines platform/application-specific configuration for the core driver test rig.
 * This test rig uses the 17HS4401S motor (gearbox or direct drive) and includes:
 * - Reference switches (endstops) for homing and limit detection
 * - Encoder (AS5047U) for position feedback
 * - Mechanical system configuration (gearbox or direct drive)
 * 
 * **Platform-specific**: These parameters depend on the application/platform:
 * - Reference switches (endstops) configuration
 * - Encoder configuration
 * - Mechanical system (gearing, leadscrew, belt drive)
 * 
 * Used by: internal_ramp_comprehensive_test, spi_daisy_chain_comprehensive_test, stallguard_tuning
 */
namespace PlatformConfig_CoreDriverTestRig {
    // Reference switch configuration
    namespace ReferenceSwitches {
        // Assuming Normally Open (NO) switches connecting to GND (Standard 3D printer style)
        // Pin states: HIGH when open (pullup), LOW when triggered (closed).
        // TMC51x0 Polarity: ACTIVE_LOW = trigger on GND, ACTIVE_HIGH = trigger on VCC.
        constexpr tmc51x0::ReferenceSwitchActiveLevel LEFT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;   ///< Left switch active level (ACTIVE_LOW for GND-triggered)
        constexpr tmc51x0::ReferenceSwitchActiveLevel RIGHT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;  ///< Right switch active level (ACTIVE_LOW for GND-triggered)
        constexpr bool LEFT_STOP_ENABLE = true;               ///< Enable motor stop on left switch
        constexpr bool RIGHT_STOP_ENABLE = true;              ///< Enable motor stop on right switch
        constexpr tmc51x0::ReferenceLatchMode LEFT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;         ///< Left switch latch mode (ACTIVE_EDGE for homing)
        constexpr tmc51x0::ReferenceLatchMode RIGHT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;          ///< Right switch latch mode (ACTIVE_EDGE for homing)
        constexpr tmc51x0::ReferenceStopMode STOP_MODE = 
            tmc51x0::ReferenceStopMode::SOFT_STOP;             ///< Stop mode (SOFT_STOP for controlled deceleration)
    }
    
    // Encoder configuration (AS5047U example)
    namespace Encoder {
        // AS5047U Specs:
        // ABI Resolution: Default 4096 ppr (pulses per rev) = 16384 counts per rev (edges)
        // SPI Resolution: 14-bit = 16384 positions
        constexpr uint16_t PULSES_PER_REV = 4096;             ///< Encoder pulses per revolution (ABI mode)
        constexpr uint16_t COUNTS_PER_REV = 16384;            ///< Encoder counts per revolution (edges)
        constexpr tmc51x0::ReferenceSwitchActiveLevel N_CHANNEL_ACTIVE = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH; ///< N channel active level
        constexpr tmc51x0::EncoderNSensitivity N_SENSITIVITY = 
            tmc51x0::EncoderNSensitivity::RISING_EDGE;        ///< N channel sensitivity (RISING_EDGE typical)
        constexpr tmc51x0::EncoderClearMode CLEAR_MODE = 
            tmc51x0::EncoderClearMode::DISABLED;               ///< Encoder clear mode (DISABLED = no clearing)
        constexpr tmc51x0::EncoderPrescalerMode PRESCALER_MODE = 
            tmc51x0::EncoderPrescalerMode::BINARY;            ///< Prescaler mode (BINARY typical)
        constexpr bool INVERT_DIRECTION = false;               ///< Invert encoder direction (false = match motor)
        
        // Encoder deviation detection
        // Maximum allowed deviation between motor position and encoder position in full steps
        // Typical range: 10-50 steps for normal operation, 1-10 steps for tight control
        constexpr int32_t ALLOWED_DEVIATION_STEPS = 25;        ///< Allowed encoder deviation in steps (0 = disabled)
    }
    
    // Mechanical system configuration
    namespace Mechanical {
        // Note: Gear ratio is typically motor-specific, but can be platform-specific if motor
        // is used with different gearboxes on different platforms. For now, gear ratio is
        // specified in MotorConfig, but can be overridden here if needed.
        constexpr tmc51x0::MechanicalSystemType SYSTEM_TYPE = 
            tmc51x0::MechanicalSystemType::Gearbox;            ///< Mechanical system type (Gearbox, DirectDrive, LeadScrew, BeltDrive)
        constexpr float LEAD_SCREW_PITCH_MM = 0.0f;           ///< Lead screw pitch in mm (0 = not used)
        constexpr uint16_t BELT_PULLEY_TEETH = 0;             ///< Number of teeth on motor pulley (0 = not used)
        constexpr float BELT_PITCH_MM = 0.0f;                 ///< Belt pitch in mm (0 = not used)
    }
}

/**
 * @brief Platform configuration for Fatigue Test Rig
 * 
 * Defines platform/application-specific configuration for the fatigue test rig.
 * This test rig uses the Applied Motion 5034-369 motor (direct drive) and includes:
 * - Reference switches (endstops) for homing and limit detection
 * - Encoder (AS5047U) for position feedback
 * - Mechanical system configuration (direct drive)
 * 
 * **Platform-specific**: These parameters depend on the application/platform:
 * - Reference switches (endstops) configuration
 * - Encoder configuration
 * - Mechanical system (gearing, leadscrew, belt drive)
 * 
 * Used by: fatigue_test_stallguard, fatigue_test_encoder, internal_ramp_sinusoidal
 */
namespace PlatformConfig_FatigueTestRig {
    // Reference switch configuration
    namespace ReferenceSwitches {
        // Assuming Normally Open (NO) switches connecting to GND (Standard 3D printer style)
        // Pin states: HIGH when open (pullup), LOW when triggered (closed).
        // TMC51x0 Polarity: ACTIVE_LOW = trigger on GND, ACTIVE_HIGH = trigger on VCC.
        constexpr tmc51x0::ReferenceSwitchActiveLevel LEFT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;   ///< Left switch active level (ACTIVE_LOW for GND-triggered)
        constexpr tmc51x0::ReferenceSwitchActiveLevel RIGHT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;  ///< Right switch active level (ACTIVE_LOW for GND-triggered)
        constexpr bool LEFT_STOP_ENABLE = true;               ///< Enable motor stop on left switch
        constexpr bool RIGHT_STOP_ENABLE = true;              ///< Enable motor stop on right switch
        constexpr tmc51x0::ReferenceLatchMode LEFT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;         ///< Left switch latch mode (ACTIVE_EDGE for homing)
        constexpr tmc51x0::ReferenceLatchMode RIGHT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;          ///< Right switch latch mode (ACTIVE_EDGE for homing)
        constexpr tmc51x0::ReferenceStopMode STOP_MODE = 
            tmc51x0::ReferenceStopMode::SOFT_STOP;             ///< Stop mode (SOFT_STOP for controlled deceleration)
    }
    
    // Encoder configuration (AS5047U example)
    namespace Encoder {
        // AS5047U Specs:
        // ABI Resolution: Default 4096 ppr (pulses per rev) = 16384 counts per rev (edges)
        // SPI Resolution: 14-bit = 16384 positions
        constexpr uint16_t PULSES_PER_REV = 4096;             ///< Encoder pulses per revolution (ABI mode)
        constexpr uint16_t COUNTS_PER_REV = 16384;            ///< Encoder counts per revolution (edges)
        constexpr tmc51x0::ReferenceSwitchActiveLevel N_CHANNEL_ACTIVE = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH; ///< N channel active level
        constexpr tmc51x0::EncoderNSensitivity N_SENSITIVITY = 
            tmc51x0::EncoderNSensitivity::RISING_EDGE;        ///< N channel sensitivity (RISING_EDGE typical)
        constexpr tmc51x0::EncoderClearMode CLEAR_MODE = 
            tmc51x0::EncoderClearMode::DISABLED;               ///< Encoder clear mode (DISABLED = no clearing)
        constexpr tmc51x0::EncoderPrescalerMode PRESCALER_MODE = 
            tmc51x0::EncoderPrescalerMode::BINARY;            ///< Prescaler mode (BINARY typical)
        constexpr bool INVERT_DIRECTION = false;               ///< Invert encoder direction (false = match motor)
        
        // Encoder deviation detection
        // Maximum allowed deviation between motor position and encoder position in full steps
        // Typical range: 10-50 steps for normal operation, 1-10 steps for tight control
        constexpr int32_t ALLOWED_DEVIATION_STEPS = 25;        ///< Allowed encoder deviation in steps (0 = disabled)
    }
    
    // Mechanical system configuration
    namespace Mechanical {
        // Fatigue test rig uses direct drive (no gearbox)
        constexpr tmc51x0::MechanicalSystemType SYSTEM_TYPE = 
            tmc51x0::MechanicalSystemType::DirectDrive;       ///< Mechanical system type (DirectDrive for Applied Motion motor)
        constexpr float LEAD_SCREW_PITCH_MM = 0.0f;           ///< Lead screw pitch in mm (0 = not used)
        constexpr uint16_t BELT_PULLEY_TEETH = 0;             ///< Number of teeth on motor pulley (0 = not used)
        constexpr float BELT_PITCH_MM = 0.0f;                 ///< Belt pitch in mm (0 = not used)
    }
}


/**
 * @brief Helper function to apply board configuration to DriverConfig
 * 
 * Applies board-specific configuration (sense resistor, supply voltage, MOSFETs, etc.)
 * to an already-configured DriverConfig. This should be called after motor configuration.
 * 
 * @param[in,out] cfg DriverConfig structure (must be configured with motor settings first)
 * @param[in] board_type Board type to use (compile-time constant)
 * 
 * @note This function configures:
 * - Sense resistor and supply voltage (for current calculation)
 * - Power stage MOSFET characteristics
 * - Short protection defaults
 * - Clock frequency
 */
template<BoardType board_type>
inline void ApplyBoardConfig(tmc51x0::DriverConfig& cfg) noexcept {
    if constexpr (board_type == BoardType::BOARD_TMC51x0_EVAL) {
        namespace Board = BoardConfig_TMC51x0_EVAL;
        
        // Driver hardware configuration (required for automatic current calculation)
        // Note: supply_voltage_mv is motor-specific and should be set from MotorConfig, not BoardConfig
        cfg.motor_spec.sense_resistor_mohm = Board::SENSE_RESISTOR_MOHM;
        
        // Power stage configuration (from board config)
        cfg.power_stage.mosfet_miller_charge_nc = Board::MOSFET_MILLER_CHARGE_NC;
        cfg.power_stage.bbm_time_ns = Board::BBM_TIME_NS;
        cfg.power_stage.sense_filter = tmc51x0::SenseFilterTime::T100ns;
        cfg.power_stage.over_temp_protection = tmc51x0::OverTempProtection::Temp150C;
        cfg.power_stage.s2vs_voltage_mv = Board::S2VS_VOLTAGE_MV;
        cfg.power_stage.s2g_voltage_mv = Board::S2G_VOLTAGE_MV;
        cfg.power_stage.shortfilter = 1;
        cfg.power_stage.short_detection_delay_us_x10 = 0;
        
        // Clock frequency (from board config)
        cfg.external_clk_config.frequency_hz = Board::CLOCK_FREQUENCY_HZ;
    }
    else if constexpr (board_type == BoardType::BOARD_TMC51x0_BOB) {
        namespace Board = BoardConfig_TMC51x0_BOB;
        
        // Driver hardware configuration (required for automatic current calculation)
        // Note: supply_voltage_mv is motor-specific and should be set from MotorConfig, not BoardConfig
        cfg.motor_spec.sense_resistor_mohm = Board::SENSE_RESISTOR_MOHM;
        
        // Power stage configuration (from board config)
        cfg.power_stage.mosfet_miller_charge_nc = Board::MOSFET_MILLER_CHARGE_NC;
        cfg.power_stage.bbm_time_ns = Board::BBM_TIME_NS;
        cfg.power_stage.sense_filter = tmc51x0::SenseFilterTime::T100ns;
        cfg.power_stage.over_temp_protection = tmc51x0::OverTempProtection::Temp150C;
        cfg.power_stage.s2vs_voltage_mv = Board::S2VS_VOLTAGE_MV;
        cfg.power_stage.s2g_voltage_mv = Board::S2G_VOLTAGE_MV;
        cfg.power_stage.shortfilter = 1;
        cfg.power_stage.short_detection_delay_us_x10 = 0;
        
        // Clock frequency (from board config)
        cfg.external_clk_config.frequency_hz = Board::CLOCK_FREQUENCY_HZ;
    }
}

/**
 * @brief Helper function to populate DriverConfig from 17HS4401S gearbox motor configuration
 * 
 * Configures motor-specific parameters only. Board and platform configuration
 * should be applied separately via ApplyBoardConfig() and ApplyPlatformConfig().
 * 
 * @param[out] cfg DriverConfig structure to populate
 */
inline void ConfigureDriverFromMotor_17HS4401S_Gearbox(tmc51x0::DriverConfig& cfg) noexcept {
    namespace Motor = MotorConfig_17HS4401S;
    
    // ===== MOTOR CONFIGURATION =====
    // Motor physical specifications
    cfg.motor_spec.steps_per_rev = Motor::MOTOR_FULL_STEPS;
    cfg.motor_spec.rated_current_ma = Motor::RATED_CURRENT_MA;
    cfg.motor_spec.supply_voltage_mv = Motor::SUPPLY_VOLTAGE_MV; // Motor-specific supply voltage
    cfg.motor_spec.winding_resistance_mohm = static_cast<uint32_t>(Motor::RESISTANCE_OHMS * 1000.0f);
    cfg.motor_spec.winding_inductance_uh = static_cast<uint32_t>(Motor::INDUCTANCE_MH * 1000.0f);
    cfg.motor_spec.run_current_ma = Motor::TARGET_RUN_CURRENT_MA;
    cfg.motor_spec.hold_current_ma = Motor::TARGET_HOLD_CURRENT_MA;
    cfg.motor_spec.iholddelay_ms = Motor::IHOLDDELAY_MS;
    
    // Motor-specific chopper configuration
    cfg.chopper.mode = tmc51x0::ChopperMode::SPREAD_CYCLE;
    cfg.chopper.toff = Motor::TOFF;
    cfg.chopper.tbl = Motor::TBL;
    cfg.chopper.hstrt = Motor::HSTRT;
    cfg.chopper.hend = Motor::HEND;
    cfg.chopper.mres = Motor::MRES;
    cfg.chopper.intpol = Motor::INTERPOLATION;
    cfg.chopper.tpfd = 0;
    
    // Motor-specific StealthChop configuration
    cfg.stealthchop.pwm_freq = Motor::STEALTH_FREQ;
    cfg.stealthchop.pwm_ofs = Motor::STEALTH_OFS;
    cfg.stealthchop.pwm_grad = 0;
    cfg.stealthchop.pwm_autoscale = Motor::STEALTH_AUTOSCALE;
    cfg.stealthchop.pwm_autograd = Motor::STEALTH_AUTOGRAD;
    cfg.stealthchop.pwm_reg = static_cast<uint8_t>(tmc51x0::StealthChopRegulationSpeed::MODERATE);
    cfg.stealthchop.pwm_lim = static_cast<uint8_t>(tmc51x0::StealthChopJerkReduction::LOW);
    cfg.stealthchop.velocity_threshold = Motor::STEALTH_VELOCITY_THRESHOLD;
    cfg.stealthchop.velocity_threshold_unit = tmc51x0::Unit::Steps;
    
    // ===== MECHANICAL SYSTEM (Motor-specific gear ratio) =====
    // Note: System type and other mechanical parameters are set via ApplyPlatformConfig()
    cfg.mechanical.gear_ratio = Motor::GEAR_RATIO;
    
    // ===== GLOBAL CONFIGURATION DEFAULTS =====
    cfg.global_config.recalibrate = false;
    cfg.global_config.en_short_standstill_timeout = false;
    cfg.global_config.en_stealthchop_mode = true;
    cfg.global_config.en_stealthchop_step_filter = true;
    cfg.direction = tmc51x0::MotorDirection::NORMAL; // Set direction in DriverConfig, shaft will be set during Initialize()
    
    // Ramp configuration defaults
    cfg.ramp_config.velocity_unit = tmc51x0::Unit::Steps;
    cfg.ramp_config.acceleration_unit = tmc51x0::Unit::Steps;
    cfg.ramp_config.vstart = Motor::RAMP_VSTART;
    cfg.ramp_config.vstop = Motor::RAMP_VSTOP;
    cfg.ramp_config.vmax = Motor::RAMP_VMAX;
    cfg.ramp_config.amax = Motor::RAMP_AMAX;
    cfg.ramp_config.dmax = Motor::RAMP_DMAX;
    cfg.ramp_config.v1 = Motor::RAMP_V1;
    cfg.ramp_config.a1 = Motor::RAMP_A1;
    cfg.ramp_config.d1 = Motor::RAMP_D1;
    cfg.ramp_config.tpowerdown_ms = Motor::RAMP_TPOWERDOWN_MS;
    cfg.ramp_config.tzerowait_ms = Motor::RAMP_TZEROWAIT_MS;
}

/**
 * @brief Helper function to populate DriverConfig from 17HS4401S direct drive motor configuration
 * 
 * Configures motor-specific parameters only. Board and platform configuration
 * should be applied separately via ApplyBoardConfig() and ApplyPlatformConfig().
 * 
 * @param[out] cfg DriverConfig structure to populate
 */
inline void ConfigureDriverFromMotor_17HS4401S_Direct(tmc51x0::DriverConfig& cfg) noexcept {
    namespace Motor = MotorConfig_17HS4401S_Direct;
    
    // ===== MOTOR CONFIGURATION =====
    cfg.motor_spec.steps_per_rev = Motor::MOTOR_FULL_STEPS;
    cfg.motor_spec.rated_current_ma = Motor::RATED_CURRENT_MA;
    cfg.motor_spec.supply_voltage_mv = Motor::SUPPLY_VOLTAGE_MV; // Motor-specific supply voltage
    cfg.motor_spec.winding_resistance_mohm = static_cast<uint32_t>(Motor::RESISTANCE_OHMS * 1000.0f);
    cfg.motor_spec.winding_inductance_uh = static_cast<uint32_t>(Motor::INDUCTANCE_MH * 1000.0f);
    cfg.motor_spec.run_current_ma = Motor::TARGET_RUN_CURRENT_MA;
    cfg.motor_spec.hold_current_ma = Motor::TARGET_HOLD_CURRENT_MA;
    cfg.motor_spec.iholddelay_ms = Motor::IHOLDDELAY_MS;
    
    cfg.chopper.mode = tmc51x0::ChopperMode::SPREAD_CYCLE;
    cfg.chopper.toff = Motor::TOFF;
    cfg.chopper.tbl = Motor::TBL;
    cfg.chopper.hstrt = Motor::HSTRT;
    cfg.chopper.hend = Motor::HEND;
    cfg.chopper.mres = Motor::MRES;
    cfg.chopper.intpol = Motor::INTERPOLATION;
    cfg.chopper.tpfd = 0;
    
    cfg.stealthchop.pwm_freq = Motor::STEALTH_FREQ;
    cfg.stealthchop.pwm_ofs = Motor::STEALTH_OFS;
    cfg.stealthchop.pwm_grad = 0;
    cfg.stealthchop.pwm_autoscale = Motor::STEALTH_AUTOSCALE;
    cfg.stealthchop.pwm_autograd = Motor::STEALTH_AUTOGRAD;
    cfg.stealthchop.pwm_reg = static_cast<uint8_t>(tmc51x0::StealthChopRegulationSpeed::MODERATE);
    cfg.stealthchop.pwm_lim = static_cast<uint8_t>(tmc51x0::StealthChopJerkReduction::LOW);
    cfg.stealthchop.velocity_threshold = Motor::STEALTH_VELOCITY_THRESHOLD;
    cfg.stealthchop.velocity_threshold_unit = tmc51x0::Unit::Steps;
    
    // ===== MECHANICAL SYSTEM (Motor-specific gear ratio) =====
    cfg.mechanical.gear_ratio = Motor::GEAR_RATIO;
    
    // ===== GLOBAL CONFIGURATION DEFAULTS =====
    cfg.global_config.recalibrate = false;
    cfg.global_config.en_short_standstill_timeout = false;
    cfg.global_config.en_stealthchop_mode = true;
    cfg.global_config.en_stealthchop_step_filter = true;
    cfg.direction = tmc51x0::MotorDirection::NORMAL; // Set direction in DriverConfig, shaft will be set during Initialize()
    
    // ===== RAMP CONFIGURATION DEFAULTS =====
    cfg.ramp_config.velocity_unit = tmc51x0::Unit::Steps;
    cfg.ramp_config.acceleration_unit = tmc51x0::Unit::Steps;
    cfg.ramp_config.vstart = Motor::RAMP_VSTART;
    cfg.ramp_config.vstop = Motor::RAMP_VSTOP;
    cfg.ramp_config.vmax = Motor::RAMP_VMAX;
    cfg.ramp_config.amax = Motor::RAMP_AMAX;
    cfg.ramp_config.dmax = Motor::RAMP_DMAX;
    cfg.ramp_config.v1 = Motor::RAMP_V1;
    cfg.ramp_config.a1 = Motor::RAMP_A1;
    cfg.ramp_config.d1 = Motor::RAMP_D1;
    cfg.ramp_config.tpowerdown_ms = Motor::RAMP_TPOWERDOWN_MS;
    cfg.ramp_config.tzerowait_ms = Motor::RAMP_TZEROWAIT_MS;
}

/**
 * @brief Helper function to populate DriverConfig from Applied Motion 5034 motor configuration
 * 
 * Configures motor-specific parameters only. Board and platform configuration
 * should be applied separately via ApplyBoardConfig() and ApplyPlatformConfig().
 * 
 * @param[out] cfg DriverConfig structure to populate
 */
inline void ConfigureDriverFromMotor_AppliedMotion_5034(tmc51x0::DriverConfig& cfg) noexcept {
    namespace Motor = MotorConfig_AppliedMotion_5034_369;
    
    // ===== MOTOR CONFIGURATION =====
    cfg.motor_spec.steps_per_rev = Motor::MOTOR_FULL_STEPS;
    cfg.motor_spec.rated_current_ma = Motor::RATED_CURRENT_MA;
    cfg.motor_spec.supply_voltage_mv = Motor::SUPPLY_VOLTAGE_MV; // Motor-specific supply voltage
    cfg.motor_spec.winding_resistance_mohm = static_cast<uint32_t>(Motor::RESISTANCE_OHMS * 1000.0f);
    cfg.motor_spec.winding_inductance_uh = static_cast<uint32_t>(Motor::INDUCTANCE_MH * 1000.0f);
    cfg.motor_spec.run_current_ma = Motor::TARGET_RUN_CURRENT_MA;
    cfg.motor_spec.hold_current_ma = Motor::TARGET_HOLD_CURRENT_MA;
    cfg.motor_spec.iholddelay_ms = Motor::IHOLDDELAY_MS;
    
    cfg.chopper.mode = tmc51x0::ChopperMode::SPREAD_CYCLE;
    cfg.chopper.toff = Motor::TOFF;
    cfg.chopper.tbl = Motor::TBL;
    cfg.chopper.hstrt = Motor::HSTRT;
    cfg.chopper.hend = Motor::HEND;
    cfg.chopper.mres = Motor::MRES;
    cfg.chopper.intpol = Motor::INTERPOLATION;
    cfg.chopper.tpfd = 0;
    
    cfg.stealthchop.pwm_freq = Motor::STEALTH_FREQ;
    cfg.stealthchop.pwm_ofs = Motor::STEALTH_OFS;
    cfg.stealthchop.pwm_grad = 0;
    cfg.stealthchop.pwm_autoscale = Motor::STEALTH_AUTOSCALE;
    cfg.stealthchop.pwm_autograd = Motor::STEALTH_AUTOGRAD;
    cfg.stealthchop.pwm_reg = static_cast<uint8_t>(tmc51x0::StealthChopRegulationSpeed::MODERATE);
    cfg.stealthchop.pwm_lim = static_cast<uint8_t>(tmc51x0::StealthChopJerkReduction::LOW);
    cfg.stealthchop.velocity_threshold = Motor::STEALTH_VELOCITY_THRESHOLD;
    cfg.stealthchop.velocity_threshold_unit = tmc51x0::Unit::Steps;
    
    // ===== MECHANICAL SYSTEM (Motor-specific gear ratio) =====
    cfg.mechanical.gear_ratio = Motor::GEAR_RATIO;
    
    // ===== GLOBAL CONFIGURATION DEFAULTS =====
    cfg.global_config.recalibrate = false;
    cfg.global_config.en_short_standstill_timeout = false;
    cfg.global_config.en_stealthchop_mode = true;
    cfg.global_config.en_stealthchop_step_filter = true;
    cfg.direction = tmc51x0::MotorDirection::NORMAL; // Set direction in DriverConfig, shaft will be set during Initialize()
    
    // ===== RAMP CONFIGURATION DEFAULTS =====
    cfg.ramp_config.velocity_unit = tmc51x0::Unit::Steps;
    cfg.ramp_config.acceleration_unit = tmc51x0::Unit::Steps;
    cfg.ramp_config.vstart = Motor::RAMP_VSTART;
    cfg.ramp_config.vstop = Motor::RAMP_VSTOP;
    cfg.ramp_config.vmax = Motor::RAMP_VMAX;
    cfg.ramp_config.amax = Motor::RAMP_AMAX;
    cfg.ramp_config.dmax = Motor::RAMP_DMAX;
    cfg.ramp_config.v1 = Motor::RAMP_V1;
    cfg.ramp_config.a1 = Motor::RAMP_A1;
    cfg.ramp_config.d1 = Motor::RAMP_D1;
    cfg.ramp_config.tpowerdown_ms = Motor::RAMP_TPOWERDOWN_MS;
    cfg.ramp_config.tzerowait_ms = Motor::RAMP_TZEROWAIT_MS;
}

/**
 * @brief Helper function to apply platform configuration to DriverConfig
 * 
 * Applies platform-specific configuration (mechanical system) to an already-configured DriverConfig.
 * This should be called after motor configuration.
 * 
 * @param[in,out] cfg DriverConfig structure (must be configured with motor settings first)
 * @param[in] platform_type Platform type to use (compile-time constant)
 * 
 * @note This function configures:
 * - Mechanical system (system type, lead screw pitch, belt parameters)
 * 
 * @note Gear ratio is set from motor config (motor-specific).
 * Reference switches and encoder are configured separately via driver methods after initialization.
 */
template<PlatformType platform_type>
inline void ApplyPlatformConfig(tmc51x0::DriverConfig& cfg) noexcept {
    if constexpr (platform_type == PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG) {
        namespace Platform = PlatformConfig_CoreDriverTestRig;
        
        // Mechanical system configuration
        cfg.mechanical.system_type = Platform::Mechanical::SYSTEM_TYPE;
        cfg.mechanical.lead_screw_pitch_mm = Platform::Mechanical::LEAD_SCREW_PITCH_MM;
        cfg.mechanical.belt_pulley_teeth = Platform::Mechanical::BELT_PULLEY_TEETH;
        cfg.mechanical.belt_pitch_mm = Platform::Mechanical::BELT_PITCH_MM;
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        namespace Platform = PlatformConfig_FatigueTestRig;
        
        // Mechanical system configuration
        cfg.mechanical.system_type = Platform::Mechanical::SYSTEM_TYPE;
        cfg.mechanical.lead_screw_pitch_mm = Platform::Mechanical::LEAD_SCREW_PITCH_MM;
        cfg.mechanical.belt_pulley_teeth = Platform::Mechanical::BELT_PULLEY_TEETH;
        cfg.mechanical.belt_pitch_mm = Platform::Mechanical::BELT_PITCH_MM;
    }
    // Add more platform types here:
    // else if constexpr (platform_type == PlatformType::PLATFORM_3D_PRINTER) {
    //     namespace Platform = PlatformConfig_3DPrinter;
    //     // ... configure from Platform namespace
    // }
}

/**
 * @brief Helper function to configure reference switches from platform config
 * 
 * Creates a ReferenceSwitchConfig structure from platform configuration.
 * This can be used after driver initialization to configure reference switches.
 * 
 * @param[in] platform_type Platform type to use (compile-time constant)
 * @return ReferenceSwitchConfig structure configured from selected platform
 */
template<PlatformType platform_type>
inline tmc51x0::ReferenceSwitchConfig GetReferenceSwitchConfig() noexcept {
    tmc51x0::ReferenceSwitchConfig ref_cfg{};
    
    if constexpr (platform_type == PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG) {
        namespace Platform = PlatformConfig_CoreDriverTestRig;
        ref_cfg.left_switch_active = Platform::ReferenceSwitches::LEFT_ACTIVE_LEVEL;
        ref_cfg.right_switch_active = Platform::ReferenceSwitches::RIGHT_ACTIVE_LEVEL;
        ref_cfg.left_switch_stop_enable = Platform::ReferenceSwitches::LEFT_STOP_ENABLE;
        ref_cfg.right_switch_stop_enable = Platform::ReferenceSwitches::RIGHT_STOP_ENABLE;
        ref_cfg.latch_left = Platform::ReferenceSwitches::LEFT_LATCH_MODE;
        ref_cfg.latch_right = Platform::ReferenceSwitches::RIGHT_LATCH_MODE;
        ref_cfg.stop_mode = Platform::ReferenceSwitches::STOP_MODE;
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        namespace Platform = PlatformConfig_FatigueTestRig;
        ref_cfg.left_switch_active = Platform::ReferenceSwitches::LEFT_ACTIVE_LEVEL;
        ref_cfg.right_switch_active = Platform::ReferenceSwitches::RIGHT_ACTIVE_LEVEL;
        ref_cfg.left_switch_stop_enable = Platform::ReferenceSwitches::LEFT_STOP_ENABLE;
        ref_cfg.right_switch_stop_enable = Platform::ReferenceSwitches::RIGHT_STOP_ENABLE;
        ref_cfg.latch_left = Platform::ReferenceSwitches::LEFT_LATCH_MODE;
        ref_cfg.latch_right = Platform::ReferenceSwitches::RIGHT_LATCH_MODE;
        ref_cfg.stop_mode = Platform::ReferenceSwitches::STOP_MODE;
    }
    // Add more platform types here:
    // else if constexpr (platform_type == PlatformType::PLATFORM_3D_PRINTER) {
    //     namespace Platform = PlatformConfig_3DPrinter;
    //     // ... configure from Platform namespace
    // }
    
    return ref_cfg;
}

/**
 * @brief Helper function to configure encoder from platform config
 * 
 * Creates an EncoderConfig structure from platform configuration.
 * This can be used after driver initialization to configure encoder.
 * 
 * @param[in] platform_type Platform type to use (compile-time constant)
 * @return EncoderConfig structure configured from selected platform
 */
template<PlatformType platform_type>
inline tmc51x0::EncoderConfig GetEncoderConfig() noexcept {
    tmc51x0::EncoderConfig enc_cfg{};
    
    if constexpr (platform_type == PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG) {
        namespace Platform = PlatformConfig_CoreDriverTestRig;
        enc_cfg.n_channel_active = Platform::Encoder::N_CHANNEL_ACTIVE;
        enc_cfg.n_sensitivity = Platform::Encoder::N_SENSITIVITY;
        enc_cfg.clear_mode = Platform::Encoder::CLEAR_MODE;
        enc_cfg.prescaler_mode = Platform::Encoder::PRESCALER_MODE;
        enc_cfg.allowed_deviation_steps = Platform::Encoder::ALLOWED_DEVIATION_STEPS;
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        namespace Platform = PlatformConfig_FatigueTestRig;
        enc_cfg.n_channel_active = Platform::Encoder::N_CHANNEL_ACTIVE;
        enc_cfg.n_sensitivity = Platform::Encoder::N_SENSITIVITY;
        enc_cfg.clear_mode = Platform::Encoder::CLEAR_MODE;
        enc_cfg.prescaler_mode = Platform::Encoder::PRESCALER_MODE;
        enc_cfg.allowed_deviation_steps = Platform::Encoder::ALLOWED_DEVIATION_STEPS;
    }
    // Add more platform types here:
    // else if constexpr (platform_type == PlatformType::PLATFORM_3D_PRINTER) {
    //     namespace Platform = PlatformConfig_3DPrinter;
    //     // ... configure from Platform namespace
    // }
    
    // Note: resolution is set separately via SetResolution() method
    return enc_cfg;
}

/**
 * @brief Helper function to get encoder pulses per revolution from platform config
 * 
 * @param[in] platform_type Platform type to use (compile-time constant)
 * @return Encoder pulses per revolution
 */
template<PlatformType platform_type>
constexpr uint16_t GetEncoderPulsesPerRev() noexcept {
    if constexpr (platform_type == PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG) {
        return PlatformConfig_CoreDriverTestRig::Encoder::PULSES_PER_REV;
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        return PlatformConfig_FatigueTestRig::Encoder::PULSES_PER_REV;
    }
    // Add more platform types here
    return 0;
}

/**
 * @brief Helper function to get encoder invert direction flag from platform config
 * 
 * @param[in] platform_type Platform type to use (compile-time constant)
 * @return true if encoder direction should be inverted
 */
template<PlatformType platform_type>
inline bool GetEncoderInvertDirection() noexcept {
    if constexpr (platform_type == PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG) {
        return PlatformConfig_CoreDriverTestRig::Encoder::INVERT_DIRECTION;
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        return PlatformConfig_FatigueTestRig::Encoder::INVERT_DIRECTION;
    }
    // Add more platform types here
    return false;
}

/**
 * @brief Get motor type for a given test rig
 * 
 * @param[in] test_rig Test rig type
 * @return Motor type for the test rig
 */
template<TestRigType test_rig>
constexpr MotorType GetTestRigMotorType() noexcept {
    if constexpr (test_rig == TestRigType::TEST_RIG_CORE_DRIVER) {
        return MotorType::MOTOR_17HS4401S_GEARBOX; // Default to gearbox, can be overridden
    }
    else if constexpr (test_rig == TestRigType::TEST_RIG_FATIGUE) {
        return MotorType::MOTOR_APPLIED_MOTION_5034;
    }
    return MotorType::MOTOR_17HS4401S_GEARBOX; // Default fallback
}

/**
 * @brief Get board type for a given test rig
 * 
 * @param[in] test_rig Test rig type
 * @return Board type for the test rig
 */
template<TestRigType test_rig>
constexpr BoardType GetTestRigBoardType() noexcept {
    if constexpr (test_rig == TestRigType::TEST_RIG_CORE_DRIVER) {
        return BoardType::BOARD_TMC51x0_EVAL;
    }
    else if constexpr (test_rig == TestRigType::TEST_RIG_FATIGUE) {
        return BoardType::BOARD_TMC51x0_EVAL;
    }
    return BoardType::BOARD_TMC51x0_EVAL; // Default fallback
}

/**
 * @brief Get platform type for a given test rig
 * 
 * @param[in] test_rig Test rig type
 * @return Platform type for the test rig
 */
template<TestRigType test_rig>
constexpr PlatformType GetTestRigPlatformType() noexcept {
    if constexpr (test_rig == TestRigType::TEST_RIG_CORE_DRIVER) {
        return PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG;
    }
    else if constexpr (test_rig == TestRigType::TEST_RIG_FATIGUE) {
        return PlatformType::PLATFORM_FATIGUE_TEST_RIG;
    }
    return PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG; // Default fallback
}

// Static storage for reference switch and encoder configs (used by DriverConfig pointers)
// These are stored per test rig type to ensure they persist for the lifetime of DriverConfig
namespace {
}

/**
 * @brief Configure driver from test rig selection
 * 
 * This is a convenience function that automatically configures motor, board, platform,
 * reference switches, and encoder based on the test rig selection.
 * For Core Driver rig, defaults to gearbox motor.
 * 
 * @param[out] cfg DriverConfig structure to populate
 * @param[in] test_rig Test rig type (compile-time constant)
 * @param[in] use_direct_drive For Core Driver rig only: true = direct drive motor, false = gearbox motor (default)
 */
template<TestRigType test_rig>
inline void ConfigureDriverFromTestRig(tmc51x0::DriverConfig& cfg, bool use_direct_drive = false) noexcept {
    // Configure motor based on test rig
    if constexpr (test_rig == TestRigType::TEST_RIG_CORE_DRIVER) {
        if (use_direct_drive) {
            ConfigureDriverFromMotor_17HS4401S_Direct(cfg);
        } else {
            ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
        }
        
        // Configure reference switches and encoder for Core Driver Test Rig
        constexpr auto platform_type = GetTestRigPlatformType<test_rig>();
        cfg.reference_switch_config = GetReferenceSwitchConfig<platform_type>();
        cfg.encoder_config = GetEncoderConfig<platform_type>();
        cfg.encoder_config.pulses_per_rev = GetEncoderPulsesPerRev<platform_type>();
        cfg.encoder_config.invert_direction = GetEncoderInvertDirection<platform_type>();
        // allowed_deviation_steps is already set in GetEncoderConfig()
    }
    else if constexpr (test_rig == TestRigType::TEST_RIG_FATIGUE) {
        ConfigureDriverFromMotor_AppliedMotion_5034(cfg);
        
        // Configure reference switches and encoder for Fatigue Test Rig
        constexpr auto platform_type = GetTestRigPlatformType<test_rig>();
        cfg.reference_switch_config = GetReferenceSwitchConfig<platform_type>();
        cfg.encoder_config = GetEncoderConfig<platform_type>();
        cfg.encoder_config.pulses_per_rev = GetEncoderPulsesPerRev<platform_type>();
        cfg.encoder_config.invert_direction = GetEncoderInvertDirection<platform_type>();
        // allowed_deviation_steps is already set in GetEncoderConfig()
    }
    
    // Apply board configuration
    ApplyBoardConfig<GetTestRigBoardType<test_rig>()>(cfg);
    
    // Apply platform configuration
    ApplyPlatformConfig<GetTestRigPlatformType<test_rig>()>(cfg);
}

/**
 * @brief Get reference switch configuration for a test rig
 * 
 * Convenience function that automatically selects the platform and returns reference switch config.
 * 
 * @param[in] test_rig Test rig type (compile-time constant)
 * @return ReferenceSwitchConfig structure configured from selected test rig
 */
template<TestRigType test_rig>
inline tmc51x0::ReferenceSwitchConfig GetTestRigReferenceSwitchConfig() noexcept {
    return GetReferenceSwitchConfig<GetTestRigPlatformType<test_rig>()>();
}

/**
 * @brief Get encoder configuration for a test rig
 * 
 * Convenience function that automatically selects the platform and returns encoder config.
 * 
 * @param[in] test_rig Test rig type (compile-time constant)
 * @return EncoderConfig structure configured from selected test rig
 */
template<TestRigType test_rig>
inline tmc51x0::EncoderConfig GetTestRigEncoderConfig() noexcept {
    return GetEncoderConfig<GetTestRigPlatformType<test_rig>()>();
}

/**
 * @brief Get encoder pulses per revolution for a test rig
 * 
 * @param[in] test_rig Test rig type (compile-time constant)
 * @return Encoder pulses per revolution
 */
template<TestRigType test_rig>
constexpr uint16_t GetTestRigEncoderPulsesPerRev() noexcept {
    return GetEncoderPulsesPerRev<GetTestRigPlatformType<test_rig>()>();
}

/**
 * @brief Get encoder invert direction flag for a test rig
 * 
 * @param[in] test_rig Test rig type (compile-time constant)
 * @return true if encoder direction should be inverted
 */
template<TestRigType test_rig>
inline bool GetTestRigEncoderInvertDirection() noexcept {
    return GetEncoderInvertDirection<GetTestRigPlatformType<test_rig>()>();
}

/**
 * @brief Get motor output full steps per revolution for a test rig
 * 
 * Returns the number of full steps per revolution at the output shaft
 * (accounting for gearbox if present).
 * 
 * @param[in] test_rig Test rig type (compile-time constant)
 * @return Output full steps per revolution
 */
template<TestRigType test_rig>
constexpr uint16_t GetTestRigMotorOutputFullSteps() noexcept {
    constexpr auto motor_type = GetTestRigMotorType<test_rig>();
    if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX) {
        return MotorConfig_17HS4401S::OUTPUT_FULL_STEPS;
    }
    else if constexpr (motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
        return MotorConfig_17HS4401S_Direct::OUTPUT_FULL_STEPS;
    }
    else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
        return MotorConfig_AppliedMotion_5034_369::OUTPUT_FULL_STEPS;
    }
    return 200; // Default fallback
}

} // namespace tmc51x0_test_config

#endif // ESP32_TMC51x0_TEST_CONFIG_HPP

