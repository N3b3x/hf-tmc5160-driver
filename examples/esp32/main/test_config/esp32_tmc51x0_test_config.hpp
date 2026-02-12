/**
 * @file esp32_tmc51x0_test_config.hpp
 * @brief ESP32 GPIO pin configuration and compile-time configuration for TMC51x0 driver (TMC5130 & TMC5160)
 *
 * This file defines compile-time configuration for TMC51x0 driver initialization using
 * struct-based configuration with static constexpr members for zero runtime overhead.
 * 
 * ## Configuration Structure
 *
 * All configurations are defined as structs with `static constexpr` members:
 * - **MotorConfig**: Motor-specific configurations (physical specs, chopper, StealthChop)
 * - **BoardConfig**: Board-specific hardware parameters (sense resistor, MOSFETs, clock)
 * - **PlatformConfig**: Platform/application-specific configuration (switches, encoder, mechanical)
 * - **TestConfig**: Test-specific parameters (StallGuard, motion profiles)
 *
 * ## Configuration Hierarchy
 *
 * 1. **BoardConfig**: Hardware parameters that stay the same for the same driver board
 *    - Sense resistor value (0.05Ω typical)
 *    - Clock frequency (12 MHz internal)
 *    - MOSFET characteristics (Miller charge, BBM time)
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
 * 4. **TestConfig**: Test-specific parameters for bounds finding and motion profiles
 *    - StallGuard thresholds and CoolStep settings
 *    - Homing speeds and timeouts
 *    - Fatigue test defaults
 *
 * ## Usage
 *
 * ### Unified Test Rig Selection (Recommended)
 *
 * The `TestRigConfig` template provides unified access to all configurations:
 *
 * ```cpp
 * // Select test rig at compile time - automatically selects motor, board, and platform
 * static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
 *     tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;
 *
 * // 1. Configure driver from test rig (automatically configures everything)
 * tmc51x0::DriverConfig cfg{};
 * tmc51x0_test_config::TestRigConfig<SELECTED_TEST_RIG>::ConfigureDriver(cfg);
 * // Or use the convenience function:
 * tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
 *
 * // 2. Initialize driver
 * driver.Initialize(cfg);
 *
 * // 3. Access test configuration values
 * using Config = tmc51x0_test_config::TestRigConfig<SELECTED_TEST_RIG>;
 * float speed = Config::Test::Motion::BOUNDS_SEARCH_SPEED_RPM;
 * int8_t sgt = Config::Test::StallGuard::SGT_HOMING;
 *
 * // 4. Get encoder and reference switch configs
 * auto ref_cfg = Config::GetReferenceSwitchConfig();
 * auto enc_cfg = Config::GetEncoderConfig();
 * driver.switches.ConfigureReferenceSwitch(ref_cfg);
 * driver.encoder.Configure(enc_cfg);
 * ```
 *
 * ### Direct Struct Access
 *
 * You can also access configuration structs directly:
 *
 * ```cpp
 * // Access motor config values
 * uint16_t steps = MotorConfig_17HS4401S::OUTPUT_FULL_STEPS;
 * float gear_ratio = MotorConfig_17HS4401S::GEAR_RATIO;
 *
 * // Access board config values
 * uint32_t sense_res = BoardConfig_TMC51x0_EVAL::SENSE_RESISTOR_MOHM;
 *
 * // Access platform config values
 * auto left_level = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::LEFT_ACTIVE_LEVEL;
 * uint16_t pulses = PlatformConfig_CoreDriverTestRig::Encoder::PULSES_PER_REV;
 *
 * // Access test config values
 * float speed = TestConfig_17HS4401S::Motion::BOUNDS_SEARCH_SPEED_RPM;
 * ```
 *
 * ### Manual Selection (Advanced)
 *
 * For advanced use cases, you can still manually select motor, board, and platform:
 *
 * ```cpp
 * tmc51x0::DriverConfig cfg{};
 * tmc51x0_test_config::ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
 * tmc51x0_test_config::ApplyBoardConfig<tmc51x0_test_config::BoardType::BOARD_TMC51x0_EVAL>(cfg);
 * tmc51x0_test_config::ApplyPlatformConfig<tmc51x0_test_config::PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG>(cfg);
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

#pragma once
#include "driver/gpio.h"
#include "tmc51x0_comm_interface.hpp"
#include "features/tmc51x0_config_builder.hpp"  // For ConfigBuilder
#include "esp32_tmc51x0_bus.hpp"  // For Esp32SpiPinConfig

namespace tmc51x0_test_config {

// ============================================================================
// GPIO PIN CONFIGURATION
// ============================================================================
// Defines all GPIO pin assignments for SPI communication and TMC51x0 control
// pins. These are used by the Esp32SPI communication interface.

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
inline Esp32SpiPinConfig GetDefaultPinConfig() noexcept {
  Esp32SpiPinConfig config{};
  
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

// ============================================================================
// TYPE ENUMERATIONS
// ============================================================================
// Enumerations for compile-time selection of motor, board, platform, and test rig.
// These types are used to select appropriate configuration namespaces at compile time.

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

// ============================================================================
// MOTOR CONFIGURATION STRUCTS
// ============================================================================
// Motor-specific parameters (physical specs, current settings, chopper, StealthChop).
// Each motor has its own struct with optimized driver settings.
// These configurations define motor physical properties and driver tuning parameters.
// All values are static constexpr members for compile-time evaluation.

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
struct MotorConfig_17HS4401S {
    // ===== Physical Motor Specs =====
    static constexpr uint16_t RATED_CURRENT_MA = 1700;  // 1.7A (17HS4401 motor specification)
    static constexpr uint32_t RESISTANCE_MOHM = 1500;   // 1.5 Ω = 1500 mΩ per phase (17HS4401 motor specification)
    static constexpr float INDUCTANCE_MH = 3.2f;        // 3.2 mH per phase (17HS4401 motor specification)
    static constexpr float GEAR_RATIO = 5.18f;          // Planetary gearbox ratio (5.18:1)
    static constexpr uint16_t MOTOR_FULL_STEPS = 200;   // Motor steps per revolution (1.8° step angle)
    // OUTPUT_FULL_STEPS: Steps per revolution at output shaft (after gearbox)
    // Calculation: MOTOR_FULL_STEPS * GEAR_RATIO = 200 * 5.18 = 1036 steps/rev
    static constexpr uint16_t OUTPUT_FULL_STEPS = static_cast<uint16_t>(MOTOR_FULL_STEPS * GEAR_RATIO); // ~1036
    static constexpr uint32_t SUPPLY_VOLTAGE_MV = 24000; // 24V (motor-specific supply voltage)

    // ===== Current Settings =====
    // Target currents (motor-specific)
    // Motor rated: 1.68A RMS. Target: 1.88A RMS (~112% rated)
    // Rationale: Slightly above rated for better StealthChop calibration and microstep performance
    // - IRUN ≥ 8 is minimum for StealthChop automatic tuning
    // - IRUN 16-31 recommended for best microstep performance
    // Hold current: ~50% of run current for energy efficiency
    static constexpr uint16_t TARGET_RUN_CURRENT_MA = 1880;  // 1.88A RMS (~112% of 1.68A rated)
    static constexpr uint16_t TARGET_HOLD_CURRENT_MA = 940;  // 0.94A RMS (~50% of run current)
    
    // ===== Microstepping Configuration =====
    // MRES=0: 256 microsteps per full step (highest resolution)
    // Calculation: Effective steps/rev = MOTOR_FULL_STEPS * 256 = 200 * 256 = 51,200 steps/rev
    // With gearbox: Effective steps/rev = OUTPUT_FULL_STEPS * 256 = 1036 * 256 = 265,216 steps/rev
    static constexpr tmc51x0::MicrostepResolution MRES = tmc51x0::MicrostepResolution::MRES_256;
    static constexpr bool INTERPOLATION = true;  // Interpolation always enabled for smoothness
    
    // ===== Chopper Configuration (SpreadCycle for NEMA 17) =====
    // TOFF: Off time (1-15). Typical: 3-5 for NEMA 17. TOFF=5 provides good balance
    // HEND: Hysteresis end (0-7). Typical: 3-5. HEND=3 provides smooth current decay
    // HSTRT: Hysteresis start (0-7). Typical: 1-5. HSTRT=4 provides good current ripple control
    // TBL: Blank time (0-3). 0=16clk, 1=24clk, 2=36clk, 3=54clk. TBL=2 (36 clocks) typical
    static constexpr uint8_t TOFF = 5;
    static constexpr uint8_t HEND = 3;
    static constexpr uint8_t HSTRT = 4;
    static constexpr uint8_t TBL = 2;  // Blank time: 36 clocks
    
    // ===== StealthChop Configuration =====
    // StealthChop provides silent operation at low speeds using PWM instead of chopper
    // STEALTH_FREQ: PWM frequency (0-3). 0=~23kHz, 1=~35kHz, 2=~47kHz, 3=~70kHz @ 12MHz clock
    // STEALTH_OFS: PWM offset (0-255). Higher = more current at standstill. Typical: 20-40
    static constexpr bool STEALTH_AUTOSCALE = true;  // Auto-calibrate PWM amplitude
    static constexpr bool STEALTH_AUTOGRAD = true;   // Auto-calibrate PWM gradient
    static constexpr uint8_t STEALTH_FREQ = 1;       // ~35kHz (good balance of smoothness and efficiency)
    static constexpr uint8_t STEALTH_OFS = 30;       // Good starting torque

    // ===== Default Ramp Profile (Tuned for NEMA 17 with gearbox) =====
    // All values in physical units for clarity and maintainability
    // With gearbox: OUTPUT_FULL_STEPS * 256 = 1036 * 256 = 265,216 steps/rev
    // RAMP_VSTART: Start velocity in RPM. Must be >0 to overcome static friction.
    // Set to 3 RPM for reliable starting (above 2 RPM minimum for testing)
    static constexpr float RAMP_VSTART_RPM = 3.0f;
    // RAMP_VSTOP: Stop velocity in RPM. Motor stops when velocity drops below this.
    // Set to 5 RPM to ensure smooth stopping above minimum test speed
    static constexpr float RAMP_VSTOP_RPM = 5.0f;
    // RAMP_VMAX: Maximum velocity in RPM. Higher max speed for NEMA 17 motors.
    // Typical NEMA 17 range: 60-300 RPM for testing. Set to 120 RPM for balanced performance.
    static constexpr float RAMP_VMAX_RPM = 120.0f;
    // RAMP_AMAX: Maximum acceleration in rev/s². Higher acceleration for lighter NEMA 17.
    // Set to reach 120 RPM in ~1 second: 120 RPM / 60 = 2 rev/s, so ~2 rev/s²
    static constexpr float RAMP_AMAX_REV_S2 = 2.0f;
    // RAMP_DMAX: Maximum deceleration in rev/s². Typically same as acceleration.
    static constexpr float RAMP_DMAX_REV_S2 = 2.0f;
    // RAMP_A1: First acceleration phase in rev/s². Used for S-curve acceleration.
    // Set to half of AMAX for smooth S-curve profile
    static constexpr float RAMP_A1_REV_S2 = 1.0f;
    // RAMP_D1: First deceleration phase in rev/s². Used for S-curve deceleration.
    static constexpr float RAMP_D1_REV_S2 = 1.0f;
    // RAMP_V1: Velocity threshold in RPM for acceleration/deceleration phase transition.
    // CRITICAL: Must be > 0 to enable two-phase acceleration and use A1/D1.
    // Set to ~30% of VMAX (36 RPM) to enable S-curve profile with smooth transition.
    // Acceleration: VSTART(3) -> V1(36) using A1(1.0), then V1(36) -> VMAX(120) using AMAX(2.0)
    // Deceleration: VMAX(120) -> V1(36) using DMAX(2.0), then V1(36) -> VSTOP(5) using D1(1.0)
    static constexpr float RAMP_V1_RPM = 36.0f;
    // RAMP_TPOWERDOWN_MS: Power down delay after standstill (ms). Motor current reduces after this time.
    static constexpr float RAMP_TPOWERDOWN_MS = 100.0f;
    // RAMP_TZEROWAIT_MS: Wait time at zero velocity before power down (ms). 0 = immediate.
    static constexpr float RAMP_TZEROWAIT_MS = 0.0f;
    
    // ===== Power Management =====
    // Motor power down delay (IHOLDDELAY)
    // Total delay time for smooth motor power down after standstill
    // Typical range: 200-500ms for smooth transition without jerk
    static constexpr float IHOLDDELAY_MS = 300.0f;  // 300ms total delay for smooth power down
    
    // StealthChop velocity threshold in RPM (TPWMTHRS)
    // Velocity below which StealthChop is active, above which SpreadCycle is used
    // Set to 0 to disable StealthChop (always use SpreadCycle)
    // Set to 30 RPM - below this StealthChop (quiet), above this SpreadCycle (more torque)
    static constexpr float STEALTH_VELOCITY_THRESHOLD_RPM = 30.0f;
};

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
struct MotorConfig_17HS4401S_Direct {
    // ===== Physical Motor Specs =====
    static constexpr uint16_t RATED_CURRENT_MA = 1700;  // 1.7A (17HS4401 motor specification)
    static constexpr uint32_t RESISTANCE_MOHM = 1500;   // 1.5 Ω = 1500 mΩ per phase (17HS4401 motor specification)
    static constexpr float INDUCTANCE_MH = 3.2f;         // 3.2 mH per phase (17HS4401 motor specification)
    static constexpr float GEAR_RATIO = 1.0f;            // Direct drive (no gearbox)
    static constexpr uint16_t MOTOR_FULL_STEPS = 200;
    static constexpr uint16_t OUTPUT_FULL_STEPS = MOTOR_FULL_STEPS; // Same as motor (no gearbox)
    static constexpr uint32_t SUPPLY_VOLTAGE_MV = 24000; // 24V (motor-specific supply voltage)

    // ===== Current Settings =====
    // Target currents (motor-specific)
    // Motor rated: 1.68A RMS. Target: 1.88A RMS (~112% rated)
    // Same as geared version since motor is identical
    // Rationale: Slightly above rated for better StealthChop calibration and microstep performance
    // Hold current: ~50% of run current for energy efficiency
    static constexpr uint16_t TARGET_RUN_CURRENT_MA = 1880;  // 1.88A RMS (~112% of 1.68A rated)
    static constexpr uint16_t TARGET_HOLD_CURRENT_MA = 940;   // 0.94A RMS (~50% of run current)
    
    // ===== Microstepping Configuration =====
    // MRES=0: 256 microsteps per full step (highest resolution)
    // Calculation: Effective steps/rev = MOTOR_FULL_STEPS * 256 = 200 * 256 = 51,200 steps/rev
    static constexpr tmc51x0::MicrostepResolution MRES = tmc51x0::MicrostepResolution::MRES_256;
    static constexpr bool INTERPOLATION = true;  // Interpolation always enabled for smoothness
    
    // ===== Chopper Configuration (SpreadCycle for NEMA 17) =====
    // TOFF: Off time (1-15). Typical: 3-5 for NEMA 17. TOFF=5 provides good balance
    // HEND: Hysteresis end (0-7). Typical: 3-5. HEND=3 provides smooth current decay
    // HSTRT: Hysteresis start (0-7). Typical: 1-5. HSTRT=4 provides good current ripple control
    // TBL: Blank time (0-3). 0=16clk, 1=24clk, 2=36clk, 3=54clk. TBL=2 (36 clocks) typical
    static constexpr uint8_t TOFF = 5;
    static constexpr uint8_t HEND = 3;
    static constexpr uint8_t HSTRT = 4;
    static constexpr uint8_t TBL = 2;  // Blank time: 36 clocks
    
    // ===== StealthChop Configuration =====
    // StealthChop provides silent operation at low speeds using PWM instead of chopper
    // STEALTH_FREQ: PWM frequency (0-3). 0=~23kHz, 1=~35kHz, 2=~47kHz, 3=~70kHz @ 12MHz clock
    // STEALTH_OFS: PWM offset (0-255). Higher = more current at standstill. Typical: 20-40
    static constexpr bool STEALTH_AUTOSCALE = true;  // Auto-calibrate PWM amplitude
    static constexpr bool STEALTH_AUTOGRAD = true;   // Auto-calibrate PWM gradient
    static constexpr uint8_t STEALTH_FREQ = 1;       // ~35kHz (good balance of smoothness and efficiency)
    static constexpr uint8_t STEALTH_OFS = 30;       // Good starting torque

    // ===== Default Ramp Profile (Tuned for NEMA 17 direct drive) =====
    // All values in physical units for clarity and maintainability
    // Direct drive: MOTOR_FULL_STEPS * 256 = 200 * 256 = 51,200 steps/rev
    // RAMP_VSTART: Start velocity in RPM. Set to 3 RPM for reliable starting (above 2 RPM minimum)
    static constexpr float RAMP_VSTART_RPM = 3.0f;
    // RAMP_VSTOP: Stop velocity in RPM. Set to 5 RPM to ensure smooth stopping above minimum test speed
    static constexpr float RAMP_VSTOP_RPM = 5.0f;
    // RAMP_VMAX: Maximum velocity in RPM. Typical NEMA 17 range: 60-300 RPM for testing.
    // Set to 120 RPM for balanced performance and torque.
    static constexpr float RAMP_VMAX_RPM = 120.0f;
    // RAMP_AMAX: Maximum acceleration in rev/s². Set to reach 120 RPM in ~1 second: 2 rev/s²
    static constexpr float RAMP_AMAX_REV_S2 = 2.0f;
    static constexpr float RAMP_DMAX_REV_S2 = 2.0f;
    // RAMP_A1: First acceleration phase in rev/s². Set to half of AMAX for smooth S-curve
    static constexpr float RAMP_A1_REV_S2 = 1.0f;
    static constexpr float RAMP_D1_REV_S2 = 1.0f;
    // RAMP_V1: Velocity threshold in RPM for acceleration/deceleration phase transition.
    // CRITICAL: Must be > 0 to enable two-phase acceleration and use A1/D1.
    // Set to ~30% of VMAX (36 RPM) to enable S-curve profile with smooth transition.
    // Acceleration: VSTART(3) -> V1(36) using A1(1.0), then V1(36) -> VMAX(120) using AMAX(2.0)
    // Deceleration: VMAX(120) -> V1(36) using DMAX(2.0), then V1(36) -> VSTOP(5) using D1(1.0)
    static constexpr float RAMP_V1_RPM = 36.0f;
    static constexpr float RAMP_TPOWERDOWN_MS = 100.0f;
    static constexpr float RAMP_TZEROWAIT_MS = 0.0f;
    
    // ===== Power Management =====
    // Motor power down delay (IHOLDDELAY)
    // Total delay time for smooth motor power down after standstill
    // Typical range: 200-500ms for smooth transition without jerk
    static constexpr float IHOLDDELAY_MS = 300.0f;  // 300ms total delay for smooth power down
    
    // StealthChop velocity threshold in RPM (TPWMTHRS)
    // Velocity below which StealthChop is active, above which SpreadCycle is used
    // Set to 0 to disable StealthChop (always use SpreadCycle)
    // Set to 30 RPM - below this StealthChop (quiet), above this SpreadCycle (more torque)
    static constexpr float STEALTH_VELOCITY_THRESHOLD_RPM = 200.0f;
};

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
struct MotorConfig_AppliedMotion_5034_369 {
    // ===== Physical Motor Specs =====
    static constexpr uint16_t RATED_CURRENT_MA = 4170;  // 4.17A RMS (bipolar series)
    static constexpr float GEAR_RATIO = 1.0f;            // Direct drive (no gearbox, 1:1 ratio)
    static constexpr uint16_t MOTOR_FULL_STEPS = 200;    // Motor steps per revolution (1.8° step angle)
    // OUTPUT_FULL_STEPS: Steps per revolution at output shaft (same as motor for direct drive)
    // Calculation: MOTOR_FULL_STEPS * GEAR_RATIO = 200 * 1.0 = 200 steps/rev
    static constexpr uint16_t OUTPUT_FULL_STEPS = MOTOR_FULL_STEPS;
    static constexpr uint32_t RESISTANCE_MOHM = 840;     // 0.84 Ω = 840 mΩ (Bipolar series resistance)
    static constexpr float INDUCTANCE_MH = 10.4f;        // Bipolar series inductance
    static constexpr uint32_t SUPPLY_VOLTAGE_MV = 24000; // 24V (motor-specific supply voltage)

    // ===== Current Settings =====
    // Target currents (motor-specific)
    // Motor rated: 4.17A RMS (bipolar series). Target: 4.17A RMS (100% rated)
    // Rationale: Use full rated current for maximum torque
    // Hold current: ~50% of run current for energy efficiency
    static constexpr uint16_t TARGET_RUN_CURRENT_MA = 4170;  // 4.17A RMS (100% of rated)
    static constexpr uint16_t TARGET_HOLD_CURRENT_MA = 2150; // 2.15A RMS (~50% of run current)
    
    // ===== Microstepping Configuration =====
    // MRES=0: 256 microsteps per full step (highest resolution)
    // Calculation: Effective steps/rev = MOTOR_FULL_STEPS * 256 = 200 * 256 = 51,200 steps/rev
    static constexpr tmc51x0::MicrostepResolution MRES = tmc51x0::MicrostepResolution::MRES_256;
    static constexpr bool INTERPOLATION = true;  // Interpolation always enabled for smoothness
    
    // ===== Chopper Configuration (SpreadCycle for NEMA 34) =====
    // TOFF: Off time (1-15). Typical: 3-5 for NEMA 34. TOFF=5 provides good balance
    // HEND: Hysteresis end (0-7). Typical: 3-5. HEND=3 provides smooth current decay
    // HSTRT: Hysteresis start (0-7). Typical: 1-5. HSTRT=4 provides good current ripple control
    // TBL: Blank time (0-3). 0=16clk, 1=24clk, 2=36clk, 3=54clk. TBL=2 (36 clocks) typical
    static constexpr uint8_t TOFF = 5;
    static constexpr uint8_t HEND = 3;
    static constexpr uint8_t HSTRT = 4;
    static constexpr uint8_t TBL = 2;  // Blank time: 36 clocks
    
    // ===== StealthChop Configuration =====
    // StealthChop provides silent operation at low speeds using PWM instead of chopper
    // STEALTH_FREQ: PWM frequency (0-3). 0=~23kHz, 1=~35kHz, 2=~47kHz, 3=~70kHz @ 12MHz clock
    // STEALTH_OFS: PWM offset (0-255). Higher = more current at standstill. Typical: 20-40
    // May need adjustment for higher current motors (NEMA 34)
    static constexpr bool STEALTH_AUTOSCALE = true;  // Auto-calibrate PWM amplitude
    static constexpr bool STEALTH_AUTOGRAD = true;   // Auto-calibrate PWM gradient
    static constexpr uint8_t STEALTH_FREQ = 1;       // ~35kHz (good balance of smoothness and efficiency)
    static constexpr uint8_t STEALTH_OFS = 30;       // May need adjustment for higher current motor

    // ===== Default Ramp Profile (Tuned for NEMA 34) =====
    // All values in physical units for clarity and maintainability
    // Direct drive: MOTOR_FULL_STEPS * 256 = 200 * 256 = 51,200 steps/rev
    // Lower acceleration due to higher rotor inertia
    // RAMP_VSTART: Start velocity in RPM. Set to 5 RPM for reliable starting (above 2 RPM minimum)
    static constexpr float RAMP_VSTART_RPM = 5.0f;
    // RAMP_VSTOP: Stop velocity in RPM. Set to 5 RPM to ensure smooth stopping above minimum test speed
    static constexpr float RAMP_VSTOP_RPM = 3.0f;
    // RAMP_VMAX: Maximum velocity in RPM. NEMA 34 typically operates 30-100 RPM for testing.
    // Set to 60 RPM for balanced performance with higher inertia motor.
    static constexpr float RAMP_VMAX_RPM = 60.0f;
    // RAMP_AMAX: Maximum acceleration in rev/s². Lower for NEMA 34 due to higher rotor inertia.
    // Set to reach 60 RPM in ~1 second: 1 rev/s²
    static constexpr float RAMP_AMAX_REV_S2 = 5.0f;
    static constexpr float RAMP_DMAX_REV_S2 = 5.0f;
    // RAMP_A1: First acceleration phase in rev/s². Set to half of AMAX for smooth S-curve
    static constexpr float RAMP_A1_REV_S2 = 2.5f;
    static constexpr float RAMP_D1_REV_S2 = 2.5f;
    // RAMP_V1: Velocity threshold in RPM for acceleration/deceleration phase transition.
    // CRITICAL: Must be > 0 to enable two-phase acceleration and use A1/D1.
    // Set to ~30% of VMAX (18 RPM) to enable S-curve profile with smooth transition.
    // Acceleration: VSTART(3) -> V1(18) using A1(0.5), then V1(18) -> VMAX(60) using AMAX(1.0)
    // Deceleration: VMAX(60) -> V1(18) using DMAX(1.0), then V1(18) -> VSTOP(5) using D1(0.5)
    static constexpr float RAMP_V1_RPM = 18.0f;
    static constexpr float RAMP_TPOWERDOWN_MS = 100.0f;
    static constexpr float RAMP_TZEROWAIT_MS = 0.0f;
    
    // ===== Power Management =====
    // Motor power down delay (IHOLDDELAY)
    // Total delay time for smooth motor power down after standstill
    // Higher values for high-torque motors to prevent mechanical shock
    // Typical range: 300-500ms for NEMA 34 motors
    static constexpr float IHOLDDELAY_MS = 400.0f;  // 400ms total delay for smooth power down
    
    // StealthChop velocity threshold in RPM (TPWMTHRS)
    // Velocity below which StealthChop is active, above which SpreadCycle is used
    // Set to 0 to disable StealthChop (always use SpreadCycle)
    // Set to 20 RPM - below this StealthChop (quiet), above this SpreadCycle (more torque for larger motor)
    static constexpr float STEALTH_VELOCITY_THRESHOLD_RPM = 500.0f;
};

/**
 * @brief Test Rig Configuration Defaults
 * 
 * Contains tuned parameters for various test scenarios.
 * These are "best guess" defaults for a typical setup with the 17HS4401S motor.
 * 
 * @note Reference switches and encoder configuration have been moved to PlatformConfig_TestRig.
 * Use PlatformConfig_TestRig::ReferenceSwitches and PlatformConfig_TestRig::Encoder instead.
 */
struct TestConfig_17HS4401S {
    // ===== Motion Profiles =====
    struct Motion {
        // **Bounds / homing search speed (RPM)**:
        // - Used by both bounds finding and sensorless homing (search phase).
        // - Must be >= `StallGuard::MIN_VELOCITY_RPM` so StallGuard is actually active (TCOOLTHRS).
        // - Higher speeds are typically more reliable for StallGuard, but increase impact energy.
        static constexpr float BOUNDS_SEARCH_SPEED_RPM = 60.0f;

        // **Bounds / homing acceleration (rev/s²)**:
        // - This is *rev/s²*, not RPM/s. Keep it modest for repeatable SG behavior.
        static constexpr float BOUNDS_SEARCH_ACCEL_REV_S2 = 5.0f;

        // **Timeout for bounds/homing search (ms)**:
        static constexpr uint32_t HOMING_TIMEOUT_MS = 30000;
    };

    // ===== Sensorless Homing / StallGuard Configuration =====
    struct StallGuard {
        // **SGT_HOMING** (StallGuard2 threshold, -64..+63):
        // - Lower (more negative) = more sensitive (stall triggers easier).
        // - Higher (more positive) = less sensitive.
        // Tune with `stallguard_tuning` at the velocity below; then validate across the window.
        static constexpr int8_t SGT_HOMING = 10;

        // **SGT tuning context**:
        // SGT is velocity dependent. Keep these values here so all apps/tools use the same assumptions.
        static constexpr float SGT_TUNED_AT_VELOCITY_RPM = Motion::BOUNDS_SEARCH_SPEED_RPM;
        static constexpr float TUNING_MIN_VELOCITY_RPM = 30.0f;
        static constexpr float TUNING_MAX_VELOCITY_RPM = 60.0f;

        // **FILTER_ENABLED (SFILT)**:
        // Enables StallGuard filtering (reduces noise/false triggers, adds latency).
        static constexpr bool FILTER_ENABLED = false;

        // **CoolStep (optional)**:
        // For sensorless homing / bounds finding we usually keep CoolStep disabled (SEMIN/SEMAX=0)
        // to avoid current modulation distorting SG readings.
        static constexpr uint8_t SEMIN = 0; // 0 = CoolStep disabled
        static constexpr uint8_t SEMAX = 0;

        // **MIN_VELOCITY_RPM (TCOOLTHRS)**:
        // StallGuard2 is only valid above this velocity (SpreadCycle only).
        static constexpr float MIN_VELOCITY_RPM = 15.0f;

        // **STALL_DETECTION_CURRENT_FACTOR**:
        // Reduces motor current during stall-detect moves. Lower current can improve sensitivity
        // and reduce mechanical stress at the endstop.
        static constexpr float STALL_DETECTION_CURRENT_FACTOR = 0.3f;  // Use 30% of rated current
    };
};

/**
 * @brief Test Configuration for Applied Motion 5034-369 Motor
 * 
 * Test-specific parameters for the Applied Motion 5034-369 NEMA 34 motor.
 * These values are optimized for the fatigue test rig.
 */
struct TestConfig_AppliedMotion_5034 {
    // ===== Motion Profiles =====
    struct Motion {
        // **Bounds / homing search speed (RPM)**:
        // For this NEMA34 fatigue rig, 60 RPM is above typical low-speed resonance and is a good
        // baseline for StallGuard bounds finding.
        static constexpr float BOUNDS_SEARCH_SPEED_RPM = 45.0f;

        // **Bounds / homing acceleration (rev/s²)**:
        static constexpr float BOUNDS_SEARCH_ACCEL_REV_S2 = 20.0f;

        // **Timeout for bounds/homing search (ms)**:
        static constexpr uint32_t HOMING_TIMEOUT_MS = 30000;
    };

    // ===== Sensorless Homing / StallGuard Configuration =====
    struct StallGuard {
        // **SGT_HOMING** (StallGuard2 threshold, -64..+63):
        // Tuned via AutoTuneStallGuard at the velocity below with current reduction.
        static constexpr int8_t SGT_HOMING = -9;

        // **SGT tuning context** (velocity dependent):
        static constexpr float SGT_TUNED_AT_VELOCITY_RPM = Motion::BOUNDS_SEARCH_SPEED_RPM;
        static constexpr float TUNING_MIN_VELOCITY_RPM = 30.0f;
        static constexpr float TUNING_MAX_VELOCITY_RPM = 80.0f;

        // **FILTER_ENABLED (SFILT)**:
        // For this rig we enable filtering to reduce false stalls during accel/decel.
        static constexpr bool FILTER_ENABLED = true;
        static constexpr uint8_t SEMIN = 0; // 0 = CoolStep disabled
        static constexpr uint8_t SEMAX = 0;

        // **MIN_VELOCITY_RPM (TCOOLTHRS)**:
        static constexpr float MIN_VELOCITY_RPM = 20.0f;

        // **STALL_DETECTION_CURRENT_FACTOR**:
        // Reduce current during stall-detect moves to improve sensitivity and reduce impact energy.
        static constexpr float STALL_DETECTION_CURRENT_FACTOR = 0.3f;  // Adjust per rig
    };
};

// ============================================================================
// BOARD CONFIGURATION NAMESPACES
// ============================================================================
// Board-specific hardware parameters (sense resistor, supply voltage, MOSFETs).
// These values are used for automatic current calculation and power stage configuration.
// Board configurations stay the same regardless of which motor is connected.

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
struct BoardConfig_TMC51x0_EVAL {
    // ===== Hardware Configuration =====
    static constexpr uint32_t SENSE_RESISTOR_MOHM = 50;      ///< Sense resistor value in milliohms (0.05Ω)
    static constexpr uint32_t CLOCK_FREQUENCY_HZ = 0;        ///< TMC51x0 clock frequency in Hz (0 = use internal 12 MHz oscillator, CLK pin tied to GND)
    
    // ===== Power Stage MOSFET Characteristics (BSC072N08NS5) =====
    static constexpr float MOSFET_MILLER_CHARGE_NC = 6.0f;   ///< MOSFET Miller charge in nC (<10nC for BSC072N08NS5)
    static constexpr uint32_t BBM_TIME_NS = 100;             ///< Break-before-make time in nanoseconds (~100ns for fast MOSFETs)
    
    // ===== Short Protection Defaults =====
    // (can be overridden per motor if needed)
    static constexpr uint16_t S2VS_VOLTAGE_MV = 625;         ///< Short to VS voltage threshold in mV (0 = auto = 625mV)
    static constexpr uint16_t S2G_VOLTAGE_MV = 625;          ///< Short to GND voltage threshold in mV (0 = auto = 625mV)
};

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
struct BoardConfig_TMC51x0_BOB {
    // ===== Hardware Configuration =====
    static constexpr uint32_t SENSE_RESISTOR_MOHM = 110;     ///< Sense resistor value in milliohms (0.11Ω - typical for BOB)
    static constexpr uint32_t CLOCK_FREQUENCY_HZ = 0;        ///< TMC51x0 clock frequency in Hz (0 = use internal 12 MHz oscillator, CLK pin tied to GND)
    
    // ===== Power Stage MOSFET Characteristics (typical BOB MOSFETs) =====
    static constexpr float MOSFET_MILLER_CHARGE_NC = 30.0f;  ///< MOSFET Miller charge in nC (~30nC for typical BOB MOSFETs)
    static constexpr uint32_t BBM_TIME_NS = 200;             ///< Break-before-make time in nanoseconds (~200ns for typical MOSFETs)
    
    // ===== Short Protection Defaults =====
    // (can be overridden per motor if needed)
    static constexpr uint16_t S2VS_VOLTAGE_MV = 625;         ///< Short to VS voltage threshold in mV (0 = auto = 625mV)
    static constexpr uint16_t S2G_VOLTAGE_MV = 625;          ///< Short to GND voltage threshold in mV (0 = auto = 625mV)
};

// ============================================================================
// PLATFORM CONFIGURATION NAMESPACES
// ============================================================================
// Platform/application-specific parameters (reference switches, encoder, mechanical system).
// These parameters depend on the application/platform and define how the motor is used
// in the physical system (endstops, position feedback, gearing, etc.).

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
struct PlatformConfig_CoreDriverTestRig {
    // ===== Reference Switch Configuration =====
    struct ReferenceSwitches {
        // Assuming Normally Open (NO) switches connecting to GND (Standard 3D printer style)
        // Pin states: HIGH when open (pullup), LOW when triggered (closed).
        // TMC51x0 Polarity: ACTIVE_LOW = trigger on GND, ACTIVE_HIGH = trigger on VCC.
        static constexpr tmc51x0::ReferenceSwitchActiveLevel LEFT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;   ///< Left switch active level (ACTIVE_LOW for GND-triggered)
        static constexpr tmc51x0::ReferenceSwitchActiveLevel RIGHT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;  ///< Right switch active level (ACTIVE_LOW for GND-triggered)
        static constexpr bool LEFT_STOP_ENABLE = true;               ///< Enable motor stop on left switch
        static constexpr bool RIGHT_STOP_ENABLE = true;              ///< Enable motor stop on right switch
        static constexpr tmc51x0::ReferenceLatchMode LEFT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;         ///< Left switch latch mode (ACTIVE_EDGE for homing)
        static constexpr tmc51x0::ReferenceLatchMode RIGHT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;          ///< Right switch latch mode (ACTIVE_EDGE for homing)
        static constexpr tmc51x0::ReferenceStopMode STOP_MODE = 
            tmc51x0::ReferenceStopMode::SOFT_STOP;             ///< Stop mode (SOFT_STOP for controlled deceleration)
    };
    
    // ===== Encoder Configuration (AS5047U example) =====
    struct Encoder {
        // AS5047U Specs:
        // ABI Resolution: Default 4096 ppr (pulses per rev) = 16384 counts per rev (edges)
        // SPI Resolution: 14-bit = 16384 positions
        static constexpr uint16_t PULSES_PER_REV = 4096;             ///< Encoder pulses per revolution (ABI mode)
        static constexpr uint16_t COUNTS_PER_REV = 16384;            ///< Encoder counts per revolution (edges)
        static constexpr tmc51x0::ReferenceSwitchActiveLevel N_CHANNEL_ACTIVE = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH; ///< N channel active level
        static constexpr tmc51x0::EncoderNSensitivity N_SENSITIVITY = 
            tmc51x0::EncoderNSensitivity::RISING_EDGE;        ///< N channel sensitivity (RISING_EDGE typical)
        static constexpr tmc51x0::EncoderClearMode CLEAR_MODE = 
            tmc51x0::EncoderClearMode::DISABLED;               ///< Encoder clear mode (DISABLED = no clearing)
        static constexpr tmc51x0::EncoderPrescalerMode PRESCALER_MODE = 
            tmc51x0::EncoderPrescalerMode::BINARY;            ///< Prescaler mode (BINARY typical)
        static constexpr bool INVERT_DIRECTION = false;               ///< Invert encoder direction (false = match motor)
        
        // Encoder deviation detection
        // Maximum allowed deviation between motor position and encoder position in full steps
        // Typical range: 10-50 steps for normal operation, 1-10 steps for tight control
        static constexpr int32_t ALLOWED_DEVIATION_STEPS = 25;        ///< Allowed encoder deviation in steps (0 = disabled)
    };
    
    // ===== Mechanical System Configuration =====
    struct Mechanical {
        // Note: Gear ratio is typically motor-specific, but can be platform-specific if motor
        // is used with different gearboxes on different platforms. For now, gear ratio is
        // specified in MotorConfig, but can be overridden here if needed.
        static constexpr tmc51x0::MechanicalSystemType SYSTEM_TYPE = 
            tmc51x0::MechanicalSystemType::Gearbox;            ///< Mechanical system type (Gearbox, DirectDrive, LeadScrew, BeltDrive)
        static constexpr float LEAD_SCREW_PITCH_MM = 0.0f;           ///< Lead screw pitch in mm (0 = not used)
        static constexpr uint16_t BELT_PULLEY_TEETH = 0;             ///< Number of teeth on motor pulley (0 = not used)
        static constexpr float BELT_PITCH_MM = 0.0f;                 ///< Belt pitch in mm (0 = not used)

        // Motor direction for this platform:
        // Use this to match the physical mounting / wiring so that "positive" motion in software
        // corresponds to the preferred mechanical direction on this rig.
        static constexpr tmc51x0::MotorDirection MOTOR_DIRECTION = tmc51x0::MotorDirection::NORMAL;
    };
};

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
struct PlatformConfig_FatigueTestRig {
    // ===== Reference Switch Configuration =====
    struct ReferenceSwitches {
        // Assuming Normally Open (NO) switches connecting to GND (Standard 3D printer style)
        // Pin states: HIGH when open (pullup), LOW when triggered (closed).
        // TMC51x0 Polarity: ACTIVE_LOW = trigger on GND, ACTIVE_HIGH = trigger on VCC.
        static constexpr tmc51x0::ReferenceSwitchActiveLevel LEFT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;   ///< Left switch active level (ACTIVE_LOW for GND-triggered)
        static constexpr tmc51x0::ReferenceSwitchActiveLevel RIGHT_ACTIVE_LEVEL = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_LOW;  ///< Right switch active level (ACTIVE_LOW for GND-triggered)
        static constexpr bool LEFT_STOP_ENABLE = true;               ///< Enable motor stop on left switch
        static constexpr bool RIGHT_STOP_ENABLE = true;              ///< Enable motor stop on right switch
        static constexpr tmc51x0::ReferenceLatchMode LEFT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;         ///< Left switch latch mode (ACTIVE_EDGE for homing)
        static constexpr tmc51x0::ReferenceLatchMode RIGHT_LATCH_MODE = 
            tmc51x0::ReferenceLatchMode::ACTIVE_EDGE;          ///< Right switch latch mode (ACTIVE_EDGE for homing)
        static constexpr tmc51x0::ReferenceStopMode STOP_MODE = 
            tmc51x0::ReferenceStopMode::SOFT_STOP;             ///< Stop mode (SOFT_STOP for controlled deceleration)
    };
    
    // ===== Encoder Configuration (AS5047U example) =====
    struct Encoder {
        // AS5047U Specs:
        // ABI Resolution: Default 4096 ppr (pulses per rev) = 16384 counts per rev (edges)
        // SPI Resolution: 14-bit = 16384 positions
        static constexpr uint16_t PULSES_PER_REV = 4096;             ///< Encoder pulses per revolution (ABI mode)
        static constexpr uint16_t COUNTS_PER_REV = 16384;            ///< Encoder counts per revolution (edges)
        static constexpr tmc51x0::ReferenceSwitchActiveLevel N_CHANNEL_ACTIVE = 
            tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH; ///< N channel active level
        static constexpr tmc51x0::EncoderNSensitivity N_SENSITIVITY = 
            tmc51x0::EncoderNSensitivity::RISING_EDGE;        ///< N channel sensitivity (RISING_EDGE typical)
        static constexpr tmc51x0::EncoderClearMode CLEAR_MODE = 
            tmc51x0::EncoderClearMode::DISABLED;               ///< Encoder clear mode (DISABLED = no clearing)
        static constexpr tmc51x0::EncoderPrescalerMode PRESCALER_MODE = 
            tmc51x0::EncoderPrescalerMode::BINARY;            ///< Prescaler mode (BINARY typical)
        static constexpr bool INVERT_DIRECTION = false;               ///< Invert encoder direction (false = match motor)
        
        // Encoder deviation detection
        // Maximum allowed deviation between motor position and encoder position in full steps
        // Typical range: 10-50 steps for normal operation, 1-10 steps for tight control
        static constexpr int32_t ALLOWED_DEVIATION_STEPS = 25;        ///< Allowed encoder deviation in steps (0 = disabled)
    };
    
    // ===== Mechanical System Configuration =====
    struct Mechanical {
        // Fatigue test rig uses direct drive (no gearbox)
        static constexpr tmc51x0::MechanicalSystemType SYSTEM_TYPE = 
            tmc51x0::MechanicalSystemType::DirectDrive;            ///< Mechanical system type (DirectDrive for Applied Motion motor)
        static constexpr float LEAD_SCREW_PITCH_MM = 0.0f;           ///< Lead screw pitch in mm (0 = not used)
        static constexpr uint16_t BELT_PULLEY_TEETH = 0;             ///< Number of teeth on motor pulley (0 = not used)
        static constexpr float BELT_PITCH_MM = 0.0f;                 ///< Belt pitch in mm (0 = not used)

        // Motor direction for this platform:
        // Flip to INVERSE if the motor is mounted/wired such that the desired "positive" direction is reversed.
        static constexpr tmc51x0::MotorDirection MOTOR_DIRECTION = tmc51x0::MotorDirection::INVERSE;
    };
};

// Forward declaration for TestRigConfig (defined after helper functions)
template<TestRigType test_rig>
struct TestRigConfig;

// ============================================================================
// CONFIGURATION HELPER FUNCTIONS
// ============================================================================
// Functions to populate DriverConfig structures from motor, board, and platform configurations.
// These functions apply configurations from the structs defined above.

/**
 * @brief Helper function to apply board configuration to DriverConfig
 * 
 * Applies board-specific configuration (sense resistor, MOSFETs, etc.)
 * to an already-configured DriverConfig. This should be called after motor configuration.
 * 
 * Uses ConfigBuilder internally for improved readability and maintainability.
 * 
 * @param[in,out] cfg DriverConfig structure (must be configured with motor settings first)
 * @param[in] board_type Board type to use (compile-time constant)
 * 
 * @note This function configures:
 * - Sense resistor (for current calculation)
 * - Power stage MOSFET characteristics
 * - Short protection defaults
 * - Clock frequency
 * 
 * @note Supply voltage is motor-specific and should be set from MotorConfig, not BoardConfig.
 */
template<BoardType board_type>
inline void ApplyBoardConfig(tmc51x0::DriverConfig& cfg) noexcept {
    // Start with existing config and apply board-specific settings
    tmc51x0::ConfigBuilder builder(cfg);
    
    if constexpr (board_type == BoardType::BOARD_TMC51x0_EVAL) {
        // TMC51x0 Evaluation Kit hardware configuration
        builder.WithSenseResistorMohm(BoardConfig_TMC51x0_EVAL::SENSE_RESISTOR_MOHM)
               .WithMosfetMillerChargeNc(BoardConfig_TMC51x0_EVAL::MOSFET_MILLER_CHARGE_NC)
               .WithBbmTimeNs(BoardConfig_TMC51x0_EVAL::BBM_TIME_NS)
               .WithSenseFilter(tmc51x0::SenseFilterTime::T100ns)
               .WithOverTempProtection(tmc51x0::OverTempProtection::Temp150C)
               .WithS2vsVoltageMv(BoardConfig_TMC51x0_EVAL::S2VS_VOLTAGE_MV)
               .WithS2gVoltageMv(BoardConfig_TMC51x0_EVAL::S2G_VOLTAGE_MV)
               .WithShortFilter(1);
        
        // Clock configuration
        if (BoardConfig_TMC51x0_EVAL::CLOCK_FREQUENCY_HZ == 0) {
            builder.WithInternalClock();
        } else {
            builder.WithExternalClockHz(BoardConfig_TMC51x0_EVAL::CLOCK_FREQUENCY_HZ);
        }
    }
    else if constexpr (board_type == BoardType::BOARD_TMC51x0_BOB) {
        // TMC51x0 Break-Out Board hardware configuration
        builder.WithSenseResistorMohm(BoardConfig_TMC51x0_BOB::SENSE_RESISTOR_MOHM)
               .WithMosfetMillerChargeNc(BoardConfig_TMC51x0_BOB::MOSFET_MILLER_CHARGE_NC)
               .WithBbmTimeNs(BoardConfig_TMC51x0_BOB::BBM_TIME_NS)
               .WithSenseFilter(tmc51x0::SenseFilterTime::T100ns)
               .WithOverTempProtection(tmc51x0::OverTempProtection::Temp150C)
               .WithS2vsVoltageMv(BoardConfig_TMC51x0_BOB::S2VS_VOLTAGE_MV)
               .WithS2gVoltageMv(BoardConfig_TMC51x0_BOB::S2G_VOLTAGE_MV)
               .WithShortFilter(1);
        
        // Clock configuration
        if (BoardConfig_TMC51x0_BOB::CLOCK_FREQUENCY_HZ == 0) {
            builder.WithInternalClock();
        } else {
            builder.WithExternalClockHz(BoardConfig_TMC51x0_BOB::CLOCK_FREQUENCY_HZ);
        }
    }
    
    cfg = builder.Build();
}

/**
 * @brief Helper function to populate DriverConfig from 17HS4401S gearbox motor configuration
 * 
 * Configures motor-specific parameters only. Board and platform configuration
 * should be applied separately via ApplyBoardConfig() and ApplyPlatformConfig().
 * 
 * Uses ConfigBuilder internally for improved readability and maintainability.
 * 
 * @param[out] cfg DriverConfig structure to populate
 */
inline void ConfigureDriverFromMotor_17HS4401S_Gearbox(tmc51x0::DriverConfig& cfg) noexcept {
    // Use ConfigBuilder for clean, readable configuration
    // Start with existing config to preserve any pre-configured values
    tmc51x0::ConfigBuilder builder(cfg);
    
    // ===== MOTOR CONFIGURATION =====
    builder.WithMotorMa(MotorConfig_17HS4401S::MOTOR_FULL_STEPS, 
                        MotorConfig_17HS4401S::RATED_CURRENT_MA)
           .WithSupplyVoltageMv(MotorConfig_17HS4401S::SUPPLY_VOLTAGE_MV)
           .WithRunCurrentMa(MotorConfig_17HS4401S::TARGET_RUN_CURRENT_MA)
           .WithHoldCurrentMa(MotorConfig_17HS4401S::TARGET_HOLD_CURRENT_MA)
           .WithWindingResistanceMohm(MotorConfig_17HS4401S::RESISTANCE_MOHM)
           .WithWindingInductanceMh(MotorConfig_17HS4401S::INDUCTANCE_MH)
           .WithMotorPowerDownDelayMs(MotorConfig_17HS4401S::IHOLDDELAY_MS)
           
           // ===== CHOPPER CONFIGURATION =====
           .WithChopperMode(tmc51x0::ChopperMode::SPREAD_CYCLE)
           .WithChopperToff(MotorConfig_17HS4401S::TOFF)
           .WithChopperBlankTime(static_cast<tmc51x0::ChopperBlankTime>(MotorConfig_17HS4401S::TBL))
           .WithHysteresisStart(MotorConfig_17HS4401S::HSTRT)
           .WithHysteresisEnd(MotorConfig_17HS4401S::HEND)
           .WithMicrostepResolution(MotorConfig_17HS4401S::MRES)
           .WithInterpolation(MotorConfig_17HS4401S::INTERPOLATION)
           
           // ===== STEALTHCHOP CONFIGURATION =====
           .WithStealthChop(true)  // Enabled via global_config.en_stealthchop_mode
           .WithStealthChopThreshold({MotorConfig_17HS4401S::STEALTH_VELOCITY_THRESHOLD_RPM, tmc51x0::Unit::RPM})
           .WithStealthChopPwmFreq(MotorConfig_17HS4401S::STEALTH_FREQ)
           .WithStealthChopPwmOfs(MotorConfig_17HS4401S::STEALTH_OFS)
           .WithStealthChopAutoscale(MotorConfig_17HS4401S::STEALTH_AUTOSCALE)
           .WithStealthChopAutograd(MotorConfig_17HS4401S::STEALTH_AUTOGRAD)
           
           // ===== GLOBAL CONFIGURATION =====
           .WithRecalibration(false)
           .WithShortStandstillTimeout(false)
           .WithStealthChopStepFilter(true)
           
           // ===== MOTION CONFIGURATION =====
           .WithStartSpeed({MotorConfig_17HS4401S::RAMP_VSTART_RPM, tmc51x0::Unit::RPM})
           .WithStopSpeed({MotorConfig_17HS4401S::RAMP_VSTOP_RPM, tmc51x0::Unit::RPM})
           .WithMaxSpeed({MotorConfig_17HS4401S::RAMP_VMAX_RPM, tmc51x0::Unit::RPM})
           .WithAcceleration({MotorConfig_17HS4401S::RAMP_AMAX_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithDeceleration({MotorConfig_17HS4401S::RAMP_DMAX_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithVelocityThreshold({MotorConfig_17HS4401S::RAMP_V1_RPM, tmc51x0::Unit::RPM})
           .WithFirstAcceleration({MotorConfig_17HS4401S::RAMP_A1_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithFirstDeceleration({MotorConfig_17HS4401S::RAMP_D1_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithRampPowerDownDelayMs(MotorConfig_17HS4401S::RAMP_TPOWERDOWN_MS)
           .WithZeroWaitTimeMs(MotorConfig_17HS4401S::RAMP_TZEROWAIT_MS)
           
           // ===== MECHANICAL SYSTEM (Motor-specific gear ratio) =====
           // Note: System type and other mechanical parameters are set via ApplyPlatformConfig()
           .WithGearbox(MotorConfig_17HS4401S::GEAR_RATIO)
           
           // ===== STALLGUARD CONFIGURATION =====
           // Set default StallGuard config from test config
           .WithStallGuardThreshold(TestConfig_17HS4401S::StallGuard::SGT_HOMING)
           .WithStallGuardFilter(TestConfig_17HS4401S::StallGuard::FILTER_ENABLED)
           .WithStallGuardMinVelocity(TestConfig_17HS4401S::StallGuard::MIN_VELOCITY_RPM, tmc51x0::Unit::RPM)
           
           // ===== DIRECTION =====
           .WithDirection(tmc51x0::MotorDirection::NORMAL);
    
    cfg = builder.Build();
}

/**
 * @brief Helper function to populate DriverConfig from 17HS4401S direct drive motor configuration
 * 
 * Configures motor-specific parameters only. Board and platform configuration
 * should be applied separately via ApplyBoardConfig() and ApplyPlatformConfig().
 * 
 * Uses ConfigBuilder internally for improved readability and maintainability.
 * 
 * @param[out] cfg DriverConfig structure to populate
 */
inline void ConfigureDriverFromMotor_17HS4401S_Direct(tmc51x0::DriverConfig& cfg) noexcept {
    // Use ConfigBuilder for clean, readable configuration
    // Start with existing config to preserve any pre-configured values
    tmc51x0::ConfigBuilder builder(cfg);
    
    // ===== MOTOR CONFIGURATION =====
    builder.WithMotorMa(MotorConfig_17HS4401S_Direct::MOTOR_FULL_STEPS, 
                        MotorConfig_17HS4401S_Direct::RATED_CURRENT_MA)
           .WithSupplyVoltageMv(MotorConfig_17HS4401S_Direct::SUPPLY_VOLTAGE_MV)
           .WithRunCurrentMa(MotorConfig_17HS4401S_Direct::TARGET_RUN_CURRENT_MA)
           .WithHoldCurrentMa(MotorConfig_17HS4401S_Direct::TARGET_HOLD_CURRENT_MA)
           .WithWindingResistanceMohm(MotorConfig_17HS4401S_Direct::RESISTANCE_MOHM)
           .WithWindingInductanceMh(MotorConfig_17HS4401S_Direct::INDUCTANCE_MH)
           .WithMotorPowerDownDelayMs(MotorConfig_17HS4401S_Direct::IHOLDDELAY_MS)
           
           // ===== CHOPPER CONFIGURATION =====
           .WithChopperMode(tmc51x0::ChopperMode::SPREAD_CYCLE)
           .WithChopperToff(MotorConfig_17HS4401S_Direct::TOFF)
           .WithChopperBlankTime(static_cast<tmc51x0::ChopperBlankTime>(MotorConfig_17HS4401S_Direct::TBL))
           .WithHysteresisStart(MotorConfig_17HS4401S_Direct::HSTRT)
           .WithHysteresisEnd(MotorConfig_17HS4401S_Direct::HEND)
           .WithMicrostepResolution(MotorConfig_17HS4401S_Direct::MRES)
           .WithInterpolation(MotorConfig_17HS4401S_Direct::INTERPOLATION)
           
           // ===== STEALTHCHOP CONFIGURATION =====
           .WithStealthChop(true)  // Enabled via global_config.en_stealthchop_mode
           .WithStealthChopThreshold({MotorConfig_17HS4401S_Direct::STEALTH_VELOCITY_THRESHOLD_RPM, tmc51x0::Unit::RPM})
           .WithStealthChopPwmFreq(MotorConfig_17HS4401S_Direct::STEALTH_FREQ)
           .WithStealthChopPwmOfs(MotorConfig_17HS4401S_Direct::STEALTH_OFS)
           .WithStealthChopAutoscale(MotorConfig_17HS4401S_Direct::STEALTH_AUTOSCALE)
           .WithStealthChopAutograd(MotorConfig_17HS4401S_Direct::STEALTH_AUTOGRAD)
           
           // ===== GLOBAL CONFIGURATION =====
           .WithRecalibration(false)
           .WithShortStandstillTimeout(false)
           .WithStealthChopStepFilter(true)
           
           // ===== MOTION CONFIGURATION =====
           .WithStartSpeed({MotorConfig_17HS4401S_Direct::RAMP_VSTART_RPM, tmc51x0::Unit::RPM})
           .WithStopSpeed({MotorConfig_17HS4401S_Direct::RAMP_VSTOP_RPM, tmc51x0::Unit::RPM})
           .WithMaxSpeed({MotorConfig_17HS4401S_Direct::RAMP_VMAX_RPM, tmc51x0::Unit::RPM})
           .WithAcceleration({MotorConfig_17HS4401S_Direct::RAMP_AMAX_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithDeceleration({MotorConfig_17HS4401S_Direct::RAMP_DMAX_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithVelocityThreshold({MotorConfig_17HS4401S_Direct::RAMP_V1_RPM, tmc51x0::Unit::RPM})
           .WithFirstAcceleration({MotorConfig_17HS4401S_Direct::RAMP_A1_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithFirstDeceleration({MotorConfig_17HS4401S_Direct::RAMP_D1_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithRampPowerDownDelayMs(MotorConfig_17HS4401S_Direct::RAMP_TPOWERDOWN_MS)
           .WithZeroWaitTimeMs(MotorConfig_17HS4401S_Direct::RAMP_TZEROWAIT_MS)
           
           // ===== MECHANICAL SYSTEM (Motor-specific gear ratio) =====
           // Note: System type and other mechanical parameters are set via ApplyPlatformConfig()
           .WithDirectDrive()  // Direct drive (gear_ratio = 1.0)
           
           // ===== STALLGUARD CONFIGURATION =====
           // Set default StallGuard config from test config
           .WithStallGuardThreshold(TestConfig_17HS4401S::StallGuard::SGT_HOMING)
           .WithStallGuardFilter(TestConfig_17HS4401S::StallGuard::FILTER_ENABLED)
           .WithStallGuardMinVelocity(TestConfig_17HS4401S::StallGuard::MIN_VELOCITY_RPM, tmc51x0::Unit::RPM)
           
           // ===== DIRECTION =====
           .WithDirection(tmc51x0::MotorDirection::NORMAL);
    
    cfg = builder.Build();
}

/**
 * @brief Helper function to populate DriverConfig from Applied Motion 5034 motor configuration
 * 
 * Configures motor-specific parameters only. Board and platform configuration
 * should be applied separately via ApplyBoardConfig() and ApplyPlatformConfig().
 * 
 * Uses ConfigBuilder internally for improved readability and maintainability.
 * 
 * @param[out] cfg DriverConfig structure to populate
 */
inline void ConfigureDriverFromMotor_AppliedMotion_5034(tmc51x0::DriverConfig& cfg) noexcept {
    // Use ConfigBuilder for clean, readable configuration
    // Start with existing config to preserve any pre-configured values
    tmc51x0::ConfigBuilder builder(cfg);
    
    // ===== MOTOR CONFIGURATION =====
    builder.WithMotorMa(MotorConfig_AppliedMotion_5034_369::MOTOR_FULL_STEPS, 
                        MotorConfig_AppliedMotion_5034_369::RATED_CURRENT_MA)
           .WithSupplyVoltageMv(MotorConfig_AppliedMotion_5034_369::SUPPLY_VOLTAGE_MV)
           .WithRunCurrentMa(MotorConfig_AppliedMotion_5034_369::TARGET_RUN_CURRENT_MA)
           .WithHoldCurrentMa(MotorConfig_AppliedMotion_5034_369::TARGET_HOLD_CURRENT_MA)
           .WithWindingResistanceMohm(MotorConfig_AppliedMotion_5034_369::RESISTANCE_MOHM)
           .WithWindingInductanceMh(MotorConfig_AppliedMotion_5034_369::INDUCTANCE_MH)
           .WithMotorPowerDownDelayMs(MotorConfig_AppliedMotion_5034_369::IHOLDDELAY_MS)
           
           // ===== CHOPPER CONFIGURATION =====
           .WithChopperMode(tmc51x0::ChopperMode::SPREAD_CYCLE)
           .WithChopperToff(MotorConfig_AppliedMotion_5034_369::TOFF)
           .WithChopperBlankTime(static_cast<tmc51x0::ChopperBlankTime>(MotorConfig_AppliedMotion_5034_369::TBL))
           .WithHysteresisStart(MotorConfig_AppliedMotion_5034_369::HSTRT)
           .WithHysteresisEnd(MotorConfig_AppliedMotion_5034_369::HEND)
           .WithMicrostepResolution(MotorConfig_AppliedMotion_5034_369::MRES)
           .WithInterpolation(MotorConfig_AppliedMotion_5034_369::INTERPOLATION)
           
           // ===== STEALTHCHOP CONFIGURATION =====
           .WithStealthChop(true)  // Enabled via global_config.en_stealthchop_mode
           .WithStealthChopThreshold({MotorConfig_AppliedMotion_5034_369::STEALTH_VELOCITY_THRESHOLD_RPM, tmc51x0::Unit::RPM})
           .WithStealthChopPwmFreq(MotorConfig_AppliedMotion_5034_369::STEALTH_FREQ)
           .WithStealthChopPwmOfs(MotorConfig_AppliedMotion_5034_369::STEALTH_OFS)
           .WithStealthChopAutoscale(MotorConfig_AppliedMotion_5034_369::STEALTH_AUTOSCALE)
           .WithStealthChopAutograd(MotorConfig_AppliedMotion_5034_369::STEALTH_AUTOGRAD)
           
           // ===== GLOBAL CONFIGURATION =====
           .WithRecalibration(false)
           .WithShortStandstillTimeout(false)
           .WithStealthChopStepFilter(true)
           
           // ===== MOTION CONFIGURATION =====
           .WithStartSpeed({MotorConfig_AppliedMotion_5034_369::RAMP_VSTART_RPM, tmc51x0::Unit::RPM})
           .WithStopSpeed({MotorConfig_AppliedMotion_5034_369::RAMP_VSTOP_RPM, tmc51x0::Unit::RPM})
           .WithMaxSpeed({MotorConfig_AppliedMotion_5034_369::RAMP_VMAX_RPM, tmc51x0::Unit::RPM})
           .WithAcceleration({MotorConfig_AppliedMotion_5034_369::RAMP_AMAX_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithDeceleration({MotorConfig_AppliedMotion_5034_369::RAMP_DMAX_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithVelocityThreshold({MotorConfig_AppliedMotion_5034_369::RAMP_V1_RPM, tmc51x0::Unit::RPM})
           .WithFirstAcceleration({MotorConfig_AppliedMotion_5034_369::RAMP_A1_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithFirstDeceleration({MotorConfig_AppliedMotion_5034_369::RAMP_D1_REV_S2, tmc51x0::Unit::RevPerSec})
           .WithRampPowerDownDelayMs(MotorConfig_AppliedMotion_5034_369::RAMP_TPOWERDOWN_MS)
           .WithZeroWaitTimeMs(MotorConfig_AppliedMotion_5034_369::RAMP_TZEROWAIT_MS)
           
           // ===== MECHANICAL SYSTEM (Motor-specific gear ratio) =====
           // Note: System type and other mechanical parameters are set via ApplyPlatformConfig()
           .WithDirectDrive()  // Direct drive (gear_ratio = 1.0)
           
           // ===== STALLGUARD CONFIGURATION =====
           // Set default StallGuard config from test config
           // This provides a sensible starting point; can be overridden at runtime (e.g., during bounds finding)
           .WithStallGuardThreshold(TestConfig_AppliedMotion_5034::StallGuard::SGT_HOMING)
           .WithStallGuardFilter(TestConfig_AppliedMotion_5034::StallGuard::FILTER_ENABLED)
           .WithStallGuardMinVelocity(TestConfig_AppliedMotion_5034::StallGuard::MIN_VELOCITY_RPM, tmc51x0::Unit::RPM)
           
           // ===== DIRECTION =====
           .WithDirection(tmc51x0::MotorDirection::NORMAL);
    
    cfg = builder.Build();
}

/**
 * @brief Helper function to apply platform configuration to DriverConfig
 * 
 * Applies platform-specific configuration (mechanical system) to an already-configured DriverConfig.
 * This should be called after motor configuration.
 * 
 * Uses ConfigBuilder internally for improved readability and maintainability.
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
    // Start with existing config and apply platform-specific mechanical system settings
    tmc51x0::ConfigBuilder builder(cfg);
    
    if constexpr (platform_type == PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG) {
        // Core Driver Test Rig mechanical system configuration
        // Gear ratio is already set from motor config, just configure system type
        if constexpr (PlatformConfig_CoreDriverTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::Gearbox) {
            builder.WithGearbox(cfg.mechanical.gear_ratio);  // Preserve gear ratio from motor config
        } else if constexpr (PlatformConfig_CoreDriverTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::DirectDrive) {
            builder.WithDirectDrive();
        } else if constexpr (PlatformConfig_CoreDriverTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::LeadScrew) {
            builder.WithLeadScrew(PlatformConfig_CoreDriverTestRig::Mechanical::LEAD_SCREW_PITCH_MM);
        } else if constexpr (PlatformConfig_CoreDriverTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::BeltDrive) {
            builder.WithBeltDrive(PlatformConfig_CoreDriverTestRig::Mechanical::BELT_PULLEY_TEETH,
                                  PlatformConfig_CoreDriverTestRig::Mechanical::BELT_PITCH_MM);
        }

        // Direction is platform-dependent (mounting/wiring). Apply it here so it can override motor defaults.
        builder.WithDirection(PlatformConfig_CoreDriverTestRig::Mechanical::MOTOR_DIRECTION);
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        // Fatigue Test Rig mechanical system configuration
        // Gear ratio is already set from motor config, just configure system type
        if constexpr (PlatformConfig_FatigueTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::Gearbox) {
            builder.WithGearbox(cfg.mechanical.gear_ratio);  // Preserve gear ratio from motor config
        } else if constexpr (PlatformConfig_FatigueTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::DirectDrive) {
            builder.WithDirectDrive();
        } else if constexpr (PlatformConfig_FatigueTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::LeadScrew) {
            builder.WithLeadScrew(PlatformConfig_FatigueTestRig::Mechanical::LEAD_SCREW_PITCH_MM);
        } else if constexpr (PlatformConfig_FatigueTestRig::Mechanical::SYSTEM_TYPE == tmc51x0::MechanicalSystemType::BeltDrive) {
            builder.WithBeltDrive(PlatformConfig_FatigueTestRig::Mechanical::BELT_PULLEY_TEETH,
                                  PlatformConfig_FatigueTestRig::Mechanical::BELT_PITCH_MM);
        }

        // Direction is platform-dependent (mounting/wiring). Apply it here so it can override motor defaults.
        builder.WithDirection(PlatformConfig_FatigueTestRig::Mechanical::MOTOR_DIRECTION);
    }
    // Add more platform types here:
    // else if constexpr (platform_type == PlatformType::PLATFORM_3D_PRINTER) {
    //     // ... configure from PlatformConfig_3DPrinter struct
    // }
    
    cfg = builder.Build();
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
        ref_cfg.left_switch_active = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::LEFT_ACTIVE_LEVEL;
        ref_cfg.right_switch_active = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::RIGHT_ACTIVE_LEVEL;
        ref_cfg.left_switch_stop_enable = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::LEFT_STOP_ENABLE;
        ref_cfg.right_switch_stop_enable = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::RIGHT_STOP_ENABLE;
        ref_cfg.latch_left = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::LEFT_LATCH_MODE;
        ref_cfg.latch_right = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::RIGHT_LATCH_MODE;
        ref_cfg.stop_mode = PlatformConfig_CoreDriverTestRig::ReferenceSwitches::STOP_MODE;
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        ref_cfg.left_switch_active = PlatformConfig_FatigueTestRig::ReferenceSwitches::LEFT_ACTIVE_LEVEL;
        ref_cfg.right_switch_active = PlatformConfig_FatigueTestRig::ReferenceSwitches::RIGHT_ACTIVE_LEVEL;
        ref_cfg.left_switch_stop_enable = PlatformConfig_FatigueTestRig::ReferenceSwitches::LEFT_STOP_ENABLE;
        ref_cfg.right_switch_stop_enable = PlatformConfig_FatigueTestRig::ReferenceSwitches::RIGHT_STOP_ENABLE;
        ref_cfg.latch_left = PlatformConfig_FatigueTestRig::ReferenceSwitches::LEFT_LATCH_MODE;
        ref_cfg.latch_right = PlatformConfig_FatigueTestRig::ReferenceSwitches::RIGHT_LATCH_MODE;
        ref_cfg.stop_mode = PlatformConfig_FatigueTestRig::ReferenceSwitches::STOP_MODE;
    }
    // Add more platform types here:
    // else if constexpr (platform_type == PlatformType::PLATFORM_3D_PRINTER) {
    //     // ... configure from PlatformConfig_3DPrinter struct
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
        enc_cfg.n_channel_active = PlatformConfig_CoreDriverTestRig::Encoder::N_CHANNEL_ACTIVE;
        enc_cfg.n_sensitivity = PlatformConfig_CoreDriverTestRig::Encoder::N_SENSITIVITY;
        enc_cfg.clear_mode = PlatformConfig_CoreDriverTestRig::Encoder::CLEAR_MODE;
        enc_cfg.prescaler_mode = PlatformConfig_CoreDriverTestRig::Encoder::PRESCALER_MODE;
        enc_cfg.allowed_deviation_steps = PlatformConfig_CoreDriverTestRig::Encoder::ALLOWED_DEVIATION_STEPS;
    }
    else if constexpr (platform_type == PlatformType::PLATFORM_FATIGUE_TEST_RIG) {
        enc_cfg.n_channel_active = PlatformConfig_FatigueTestRig::Encoder::N_CHANNEL_ACTIVE;
        enc_cfg.n_sensitivity = PlatformConfig_FatigueTestRig::Encoder::N_SENSITIVITY;
        enc_cfg.clear_mode = PlatformConfig_FatigueTestRig::Encoder::CLEAR_MODE;
        enc_cfg.prescaler_mode = PlatformConfig_FatigueTestRig::Encoder::PRESCALER_MODE;
        enc_cfg.allowed_deviation_steps = PlatformConfig_FatigueTestRig::Encoder::ALLOWED_DEVIATION_STEPS;
    }
    // Add more platform types here:
    // else if constexpr (platform_type == PlatformType::PLATFORM_3D_PRINTER) {
    //     // ... configure from PlatformConfig_3DPrinter struct
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

/**
 * @brief Test configuration accessor struct
 * 
 * Provides compile-time access to test configuration parameters based on motor type.
 * This struct wraps the test config namespace values to allow template-based selection.
 */
template<MotorType motor_type>
struct TestConfigAccessor {
    // StallGuard configuration
    struct StallGuard {
        static constexpr int8_t SGT_HOMING = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::StallGuard::SGT_HOMING;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::StallGuard::SGT_HOMING;
            }
            return int8_t(10); // Default fallback
        }();
        
        static constexpr bool FILTER_ENABLED = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::StallGuard::FILTER_ENABLED;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::StallGuard::FILTER_ENABLED;
            }
            return true; // Default fallback
        }();
        
        static constexpr uint8_t SEMIN = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::StallGuard::SEMIN;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::StallGuard::SEMIN;
            }
            return uint8_t(2); // Default fallback
        }();
        
        static constexpr uint8_t SEMAX = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::StallGuard::SEMAX;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::StallGuard::SEMAX;
            }
            return uint8_t(5); // Default fallback
        }();
        
        // Minimum velocity (TCOOLTHRS) for StallGuard2 operation in RPM
        static constexpr float MIN_VELOCITY_RPM = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::StallGuard::MIN_VELOCITY_RPM;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::StallGuard::MIN_VELOCITY_RPM;
            }
            return 60.0f; // Default fallback
        }();
    };
    
    // Motion profile configuration
    struct Motion {
        // HOMING_SEARCH_SPEED_RPM removed - use BOUNDS_SEARCH_SPEED_RPM for all search operations
        // (consolidated since both are search speeds for finding limits)
        // HOMING_SWITCH_SPEED_RPM removed - not used in current implementation
        
        static constexpr float BOUNDS_SEARCH_SPEED_RPM = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::Motion::BOUNDS_SEARCH_SPEED_RPM;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::Motion::BOUNDS_SEARCH_SPEED_RPM;
            }
            return 30.0f; // Default fallback
        }();
        
        static constexpr float BOUNDS_SEARCH_ACCEL_REV_S2 = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::Motion::BOUNDS_SEARCH_ACCEL_REV_S2;
            }
            return 5; // Default fallback
        }();
        
        static constexpr uint32_t HOMING_TIMEOUT_MS = []() {
            if constexpr (motor_type == MotorType::MOTOR_17HS4401S_GEARBOX || 
                          motor_type == MotorType::MOTOR_17HS4401S_DIRECT) {
                return TestConfig_17HS4401S::Motion::HOMING_TIMEOUT_MS;
            }
            else if constexpr (motor_type == MotorType::MOTOR_APPLIED_MOTION_5034) {
                return TestConfig_AppliedMotion_5034::Motion::HOMING_TIMEOUT_MS;
            }
            return uint32_t(30000); // Default fallback
        }();
    };
};

/**
 * @brief Get test configuration accessor for a given test rig
 * 
 * Automatically selects the appropriate test config based on the test rig's motor type.
 * This provides compile-time access to test configuration parameters.
 * 
 * Usage:
 *   using TestConfig = GetTestConfigForTestRig<SELECTED_TEST_RIG>;
 *   float speed = TestConfig::Motion::BOUNDS_SEARCH_SPEED_RPM;
 * 
 * @note This is now an alias to TestRigConfig::Test for consistency.
 *       The TestConfigAccessor is still available for backward compatibility.
 */
template<TestRigType test_rig>
using GetTestConfigForTestRig = typename TestRigConfig<test_rig>::Test;

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
    TestRigConfig<test_rig>::ConfigureDriver(cfg, use_direct_drive);
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
 * @param[in] use_direct_drive For Core Driver rig only: true = direct drive motor, false = gearbox motor (default)
 * @return Output full steps per revolution
 * 
 * @note This function now delegates to TestRigConfig::GetMotorOutputFullSteps() for consistency.
 */
template<TestRigType test_rig>
constexpr uint16_t GetTestRigMotorOutputFullSteps(bool use_direct_drive = false) noexcept {
    return TestRigConfig<test_rig>::GetMotorOutputFullSteps(use_direct_drive);
}

// ============================================================================
// TEST RIG CONFIGURATION TEMPLATE
// ============================================================================
// Unified template for accessing all configurations (Motor, Board, Platform, Test) for a test rig.
// This provides compile-time type-safe access to all configuration values.
// NOTE: This is defined after helper functions so they are in scope.

/**
 * @brief Base template for test rig configuration (undefined - forces explicit specialization)
 * 
 * Each test rig must provide an explicit specialization that defines:
 * - Motor config type alias
 * - Board config type alias
 * - Platform config type alias
 * - Test config type alias
 * - Helper functions for driver configuration
 */
template<TestRigType test_rig>
struct TestRigConfig;

/**
 * @brief Test rig configuration for TEST_RIG_CORE_DRIVER
 * 
 * Uses 17HS4401S motor (gearbox by default), TMC51x0_EVAL board, and CoreDriverTestRig platform.
 */
template<>
struct TestRigConfig<TestRigType::TEST_RIG_CORE_DRIVER> {
    using Motor = MotorConfig_17HS4401S;  // Default to gearbox
    using Board = BoardConfig_TMC51x0_EVAL;
    using Platform = PlatformConfig_CoreDriverTestRig;
    using Test = TestConfig_17HS4401S;
    
    // Type info
    static constexpr MotorType motor_type = MotorType::MOTOR_17HS4401S_GEARBOX;
    static constexpr BoardType board_type = BoardType::BOARD_TMC51x0_EVAL;
    static constexpr PlatformType platform_type = PlatformType::PLATFORM_CORE_DRIVER_TEST_RIG;
    
    /**
     * @brief Configure driver from this test rig's configuration
     * 
     * @param[out] cfg DriverConfig structure to populate
     * @param[in] use_direct_drive If true, use direct drive motor instead of gearbox
     */
    static void ConfigureDriver(tmc51x0::DriverConfig& cfg, bool use_direct_drive = false) noexcept {
        if (use_direct_drive) {
            ConfigureDriverFromMotor_17HS4401S_Direct(cfg);
        } else {
            ConfigureDriverFromMotor_17HS4401S_Gearbox(cfg);
        }
        
        ApplyBoardConfig<board_type>(cfg);
        ApplyPlatformConfig<platform_type>(cfg);
        
        // Configure reference switches and encoder
        cfg.reference_switch_config = tmc51x0_test_config::GetReferenceSwitchConfig<platform_type>();
        cfg.encoder_config = tmc51x0_test_config::GetEncoderConfig<platform_type>();
        cfg.encoder_config.pulses_per_rev = tmc51x0_test_config::GetEncoderPulsesPerRev<platform_type>();
        cfg.encoder_config.invert_direction = tmc51x0_test_config::GetEncoderInvertDirection<platform_type>();
    }
    
    /**
     * @brief Get motor output full steps per revolution
     * 
     * @param[in] use_direct_drive If true, use direct drive motor instead of gearbox
     * @return Full steps per revolution at output shaft
     */
    static constexpr uint16_t GetMotorOutputFullSteps(bool use_direct_drive = false) noexcept {
        if (use_direct_drive) {
            return MotorConfig_17HS4401S_Direct::OUTPUT_FULL_STEPS;
        } else {
            return Motor::OUTPUT_FULL_STEPS;
        }
    }
    
    /**
     * @brief Get encoder configuration
     * 
     * @return EncoderConfig structure
     */
    static constexpr tmc51x0::EncoderConfig GetEncoderConfig() noexcept {
        return tmc51x0_test_config::GetEncoderConfig<platform_type>();
    }
    
    /**
     * @brief Get reference switch configuration
     * 
     * @return ReferenceSwitchConfig structure
     */
    static constexpr tmc51x0::ReferenceSwitchConfig GetReferenceSwitchConfig() noexcept {
        return tmc51x0_test_config::GetReferenceSwitchConfig<platform_type>();
    }
};

/**
 * @brief Test rig configuration for TEST_RIG_FATIGUE
 * 
 * Uses Applied Motion 5034-369 motor, TMC51x0_EVAL board, and FatigueTestRig platform.
 */
template<>
struct TestRigConfig<TestRigType::TEST_RIG_FATIGUE> {
    using Motor = MotorConfig_AppliedMotion_5034_369;
    using Board = BoardConfig_TMC51x0_EVAL;
    using Platform = PlatformConfig_FatigueTestRig;
    using Test = TestConfig_AppliedMotion_5034;
    
    // Type info
    static constexpr MotorType motor_type = MotorType::MOTOR_APPLIED_MOTION_5034;
    static constexpr BoardType board_type = BoardType::BOARD_TMC51x0_EVAL;
    static constexpr PlatformType platform_type = PlatformType::PLATFORM_FATIGUE_TEST_RIG;
    
    /**
     * @brief Configure driver from this test rig's configuration
     * 
     * @param[out] cfg DriverConfig structure to populate
     * @param[in] use_direct_drive Ignored (this motor is always direct drive)
     */
    static void ConfigureDriver(tmc51x0::DriverConfig& cfg, bool use_direct_drive = false) noexcept {
        (void)use_direct_drive; // Unused parameter
        ConfigureDriverFromMotor_AppliedMotion_5034(cfg);
        
        ApplyBoardConfig<board_type>(cfg);
        ApplyPlatformConfig<platform_type>(cfg);
        
        // Configure reference switches and encoder
        cfg.reference_switch_config = tmc51x0_test_config::GetReferenceSwitchConfig<platform_type>();
        cfg.encoder_config = tmc51x0_test_config::GetEncoderConfig<platform_type>();
        cfg.encoder_config.pulses_per_rev = tmc51x0_test_config::GetEncoderPulsesPerRev<platform_type>();
        cfg.encoder_config.invert_direction = tmc51x0_test_config::GetEncoderInvertDirection<platform_type>();
    }
    
    /**
     * @brief Get motor output full steps per revolution
     * 
     * @param[in] use_direct_drive Ignored (this motor is always direct drive)
     * @return Full steps per revolution at output shaft
     */
    static constexpr uint16_t GetMotorOutputFullSteps(bool use_direct_drive = false) noexcept {
        (void)use_direct_drive; // Unused parameter
        return Motor::OUTPUT_FULL_STEPS;
    }
    
    /**
     * @brief Get encoder configuration
     * 
     * @return EncoderConfig structure
     */
    static constexpr tmc51x0::EncoderConfig GetEncoderConfig() noexcept {
        return tmc51x0_test_config::GetEncoderConfig<platform_type>();
    }
    
    /**
     * @brief Get reference switch configuration
     * 
     * @return ReferenceSwitchConfig structure
     */
    static constexpr tmc51x0::ReferenceSwitchConfig GetReferenceSwitchConfig() noexcept {
        return tmc51x0_test_config::GetReferenceSwitchConfig<platform_type>();
    }
};

// ============================================================================
// COMPILE-TIME CONFIGURATION VALIDATORS
// ============================================================================
// Validators to ensure all required configuration values exist at compile time.
// These use C++20 requires expressions to check for required members.

/**
 * @brief Validator helper - checks if a type has required members
 * 
 * Uses C++20 requires expressions to verify all required members exist.
 * If any member is missing, compilation will fail with a clear error.
 */
namespace ConfigValidators {
    // Validate MotorConfig_17HS4401S
    static_assert(requires {
        MotorConfig_17HS4401S::RATED_CURRENT_MA;
        MotorConfig_17HS4401S::GEAR_RATIO;
        MotorConfig_17HS4401S::MOTOR_FULL_STEPS;
        MotorConfig_17HS4401S::OUTPUT_FULL_STEPS;
        MotorConfig_17HS4401S::SUPPLY_VOLTAGE_MV;
        MotorConfig_17HS4401S::TARGET_RUN_CURRENT_MA;
        MotorConfig_17HS4401S::TARGET_HOLD_CURRENT_MA;
        MotorConfig_17HS4401S::MRES;
        MotorConfig_17HS4401S::INTERPOLATION;
        MotorConfig_17HS4401S::TOFF;
        MotorConfig_17HS4401S::HEND;
        MotorConfig_17HS4401S::HSTRT;
        MotorConfig_17HS4401S::TBL;
        MotorConfig_17HS4401S::STEALTH_AUTOSCALE;
        MotorConfig_17HS4401S::STEALTH_AUTOGRAD;
        MotorConfig_17HS4401S::STEALTH_FREQ;
        MotorConfig_17HS4401S::STEALTH_OFS;
        MotorConfig_17HS4401S::RAMP_VSTART_RPM;
        MotorConfig_17HS4401S::RAMP_VSTOP_RPM;
        MotorConfig_17HS4401S::RAMP_VMAX_RPM;
        MotorConfig_17HS4401S::RAMP_AMAX_REV_S2;
        MotorConfig_17HS4401S::RAMP_DMAX_REV_S2;
        MotorConfig_17HS4401S::IHOLDDELAY_MS;
        MotorConfig_17HS4401S::STEALTH_VELOCITY_THRESHOLD_RPM;
    }, "MotorConfig_17HS4401S is missing required members");

    // Validate MotorConfig_17HS4401S_Direct
    static_assert(requires {
        MotorConfig_17HS4401S_Direct::RATED_CURRENT_MA;
        MotorConfig_17HS4401S_Direct::GEAR_RATIO;
        MotorConfig_17HS4401S_Direct::MOTOR_FULL_STEPS;
        MotorConfig_17HS4401S_Direct::OUTPUT_FULL_STEPS;
        MotorConfig_17HS4401S_Direct::SUPPLY_VOLTAGE_MV;
        MotorConfig_17HS4401S_Direct::TARGET_RUN_CURRENT_MA;
        MotorConfig_17HS4401S_Direct::TARGET_HOLD_CURRENT_MA;
        MotorConfig_17HS4401S_Direct::MRES;
        MotorConfig_17HS4401S_Direct::INTERPOLATION;
        MotorConfig_17HS4401S_Direct::TOFF;
        MotorConfig_17HS4401S_Direct::HEND;
        MotorConfig_17HS4401S_Direct::HSTRT;
        MotorConfig_17HS4401S_Direct::TBL;
        MotorConfig_17HS4401S_Direct::STEALTH_AUTOSCALE;
        MotorConfig_17HS4401S_Direct::STEALTH_AUTOGRAD;
        MotorConfig_17HS4401S_Direct::STEALTH_FREQ;
        MotorConfig_17HS4401S_Direct::STEALTH_OFS;
        MotorConfig_17HS4401S_Direct::RAMP_VSTART_RPM;
        MotorConfig_17HS4401S_Direct::RAMP_VSTOP_RPM;
        MotorConfig_17HS4401S_Direct::RAMP_VMAX_RPM;
        MotorConfig_17HS4401S_Direct::RAMP_AMAX_REV_S2;
        MotorConfig_17HS4401S_Direct::RAMP_DMAX_REV_S2;
        MotorConfig_17HS4401S_Direct::IHOLDDELAY_MS;
        MotorConfig_17HS4401S_Direct::STEALTH_VELOCITY_THRESHOLD_RPM;
    }, "MotorConfig_17HS4401S_Direct is missing required members");

    // Validate MotorConfig_AppliedMotion_5034_369
    static_assert(requires {
        MotorConfig_AppliedMotion_5034_369::RATED_CURRENT_MA;
        MotorConfig_AppliedMotion_5034_369::GEAR_RATIO;
        MotorConfig_AppliedMotion_5034_369::MOTOR_FULL_STEPS;
        MotorConfig_AppliedMotion_5034_369::OUTPUT_FULL_STEPS;
        MotorConfig_AppliedMotion_5034_369::SUPPLY_VOLTAGE_MV;
        MotorConfig_AppliedMotion_5034_369::TARGET_RUN_CURRENT_MA;
        MotorConfig_AppliedMotion_5034_369::TARGET_HOLD_CURRENT_MA;
        MotorConfig_AppliedMotion_5034_369::MRES;
        MotorConfig_AppliedMotion_5034_369::INTERPOLATION;
        MotorConfig_AppliedMotion_5034_369::TOFF;
        MotorConfig_AppliedMotion_5034_369::HEND;
        MotorConfig_AppliedMotion_5034_369::HSTRT;
        MotorConfig_AppliedMotion_5034_369::TBL;
        MotorConfig_AppliedMotion_5034_369::STEALTH_AUTOSCALE;
        MotorConfig_AppliedMotion_5034_369::STEALTH_AUTOGRAD;
        MotorConfig_AppliedMotion_5034_369::STEALTH_FREQ;
        MotorConfig_AppliedMotion_5034_369::STEALTH_OFS;
        MotorConfig_AppliedMotion_5034_369::RAMP_VSTART_RPM;
        MotorConfig_AppliedMotion_5034_369::RAMP_VSTOP_RPM;
        MotorConfig_AppliedMotion_5034_369::RAMP_VMAX_RPM;
        MotorConfig_AppliedMotion_5034_369::RAMP_AMAX_REV_S2;
        MotorConfig_AppliedMotion_5034_369::RAMP_DMAX_REV_S2;
        MotorConfig_AppliedMotion_5034_369::IHOLDDELAY_MS;
        MotorConfig_AppliedMotion_5034_369::STEALTH_VELOCITY_THRESHOLD_RPM;
    }, "MotorConfig_AppliedMotion_5034_369 is missing required members");

    // Validate BoardConfig_TMC51x0_EVAL
    static_assert(requires {
        BoardConfig_TMC51x0_EVAL::SENSE_RESISTOR_MOHM;
        BoardConfig_TMC51x0_EVAL::CLOCK_FREQUENCY_HZ;
        BoardConfig_TMC51x0_EVAL::MOSFET_MILLER_CHARGE_NC;
        BoardConfig_TMC51x0_EVAL::BBM_TIME_NS;
        BoardConfig_TMC51x0_EVAL::S2VS_VOLTAGE_MV;
        BoardConfig_TMC51x0_EVAL::S2G_VOLTAGE_MV;
    }, "BoardConfig_TMC51x0_EVAL is missing required members");

    // Validate BoardConfig_TMC51x0_BOB
    static_assert(requires {
        BoardConfig_TMC51x0_BOB::SENSE_RESISTOR_MOHM;
        BoardConfig_TMC51x0_BOB::CLOCK_FREQUENCY_HZ;
        BoardConfig_TMC51x0_BOB::MOSFET_MILLER_CHARGE_NC;
        BoardConfig_TMC51x0_BOB::BBM_TIME_NS;
        BoardConfig_TMC51x0_BOB::S2VS_VOLTAGE_MV;
        BoardConfig_TMC51x0_BOB::S2G_VOLTAGE_MV;
    }, "BoardConfig_TMC51x0_BOB is missing required members");

    // Validate PlatformConfig_CoreDriverTestRig
    static_assert(requires {
        PlatformConfig_CoreDriverTestRig::ReferenceSwitches::LEFT_ACTIVE_LEVEL;
        PlatformConfig_CoreDriverTestRig::ReferenceSwitches::RIGHT_ACTIVE_LEVEL;
        PlatformConfig_CoreDriverTestRig::ReferenceSwitches::LEFT_STOP_ENABLE;
        PlatformConfig_CoreDriverTestRig::ReferenceSwitches::RIGHT_STOP_ENABLE;
        PlatformConfig_CoreDriverTestRig::ReferenceSwitches::LEFT_LATCH_MODE;
        PlatformConfig_CoreDriverTestRig::ReferenceSwitches::RIGHT_LATCH_MODE;
        PlatformConfig_CoreDriverTestRig::ReferenceSwitches::STOP_MODE;
        PlatformConfig_CoreDriverTestRig::Encoder::PULSES_PER_REV;
        PlatformConfig_CoreDriverTestRig::Encoder::COUNTS_PER_REV;
        PlatformConfig_CoreDriverTestRig::Encoder::N_CHANNEL_ACTIVE;
        PlatformConfig_CoreDriverTestRig::Encoder::N_SENSITIVITY;
        PlatformConfig_CoreDriverTestRig::Encoder::CLEAR_MODE;
        PlatformConfig_CoreDriverTestRig::Encoder::PRESCALER_MODE;
        PlatformConfig_CoreDriverTestRig::Encoder::INVERT_DIRECTION;
        PlatformConfig_CoreDriverTestRig::Encoder::ALLOWED_DEVIATION_STEPS;
        PlatformConfig_CoreDriverTestRig::Mechanical::SYSTEM_TYPE;
        PlatformConfig_CoreDriverTestRig::Mechanical::MOTOR_DIRECTION;
        PlatformConfig_CoreDriverTestRig::Mechanical::LEAD_SCREW_PITCH_MM;
        PlatformConfig_CoreDriverTestRig::Mechanical::BELT_PULLEY_TEETH;
        PlatformConfig_CoreDriverTestRig::Mechanical::BELT_PITCH_MM;
    }, "PlatformConfig_CoreDriverTestRig is missing required members");

    // Validate PlatformConfig_FatigueTestRig
    static_assert(requires {
        PlatformConfig_FatigueTestRig::ReferenceSwitches::LEFT_ACTIVE_LEVEL;
        PlatformConfig_FatigueTestRig::ReferenceSwitches::RIGHT_ACTIVE_LEVEL;
        PlatformConfig_FatigueTestRig::ReferenceSwitches::LEFT_STOP_ENABLE;
        PlatformConfig_FatigueTestRig::ReferenceSwitches::RIGHT_STOP_ENABLE;
        PlatformConfig_FatigueTestRig::ReferenceSwitches::LEFT_LATCH_MODE;
        PlatformConfig_FatigueTestRig::ReferenceSwitches::RIGHT_LATCH_MODE;
        PlatformConfig_FatigueTestRig::ReferenceSwitches::STOP_MODE;
        PlatformConfig_FatigueTestRig::Encoder::PULSES_PER_REV;
        PlatformConfig_FatigueTestRig::Encoder::COUNTS_PER_REV;
        PlatformConfig_FatigueTestRig::Encoder::N_CHANNEL_ACTIVE;
        PlatformConfig_FatigueTestRig::Encoder::N_SENSITIVITY;
        PlatformConfig_FatigueTestRig::Encoder::CLEAR_MODE;
        PlatformConfig_FatigueTestRig::Encoder::PRESCALER_MODE;
        PlatformConfig_FatigueTestRig::Encoder::INVERT_DIRECTION;
        PlatformConfig_FatigueTestRig::Encoder::ALLOWED_DEVIATION_STEPS;
        PlatformConfig_FatigueTestRig::Mechanical::SYSTEM_TYPE;
        PlatformConfig_FatigueTestRig::Mechanical::MOTOR_DIRECTION;
        PlatformConfig_FatigueTestRig::Mechanical::LEAD_SCREW_PITCH_MM;
        PlatformConfig_FatigueTestRig::Mechanical::BELT_PULLEY_TEETH;
        PlatformConfig_FatigueTestRig::Mechanical::BELT_PITCH_MM;
    }, "PlatformConfig_FatigueTestRig is missing required members");

    // Validate TestConfig_17HS4401S
    static_assert(requires {
        TestConfig_17HS4401S::StallGuard::SGT_HOMING;
        TestConfig_17HS4401S::StallGuard::FILTER_ENABLED;
        TestConfig_17HS4401S::StallGuard::SEMIN;
        TestConfig_17HS4401S::StallGuard::SEMAX;
        TestConfig_17HS4401S::StallGuard::MIN_VELOCITY_RPM;
        TestConfig_17HS4401S::Motion::BOUNDS_SEARCH_SPEED_RPM;
        TestConfig_17HS4401S::Motion::HOMING_TIMEOUT_MS;
    }, "TestConfig_17HS4401S is missing required members");

    // Validate TestConfig_AppliedMotion_5034
    static_assert(requires {
        TestConfig_AppliedMotion_5034::StallGuard::SGT_HOMING;
        TestConfig_AppliedMotion_5034::StallGuard::FILTER_ENABLED;
        TestConfig_AppliedMotion_5034::StallGuard::SEMIN;
        TestConfig_AppliedMotion_5034::StallGuard::SEMAX;
        TestConfig_AppliedMotion_5034::StallGuard::MIN_VELOCITY_RPM;
        TestConfig_AppliedMotion_5034::Motion::BOUNDS_SEARCH_SPEED_RPM;
        TestConfig_AppliedMotion_5034::Motion::HOMING_TIMEOUT_MS;
    }, "TestConfig_AppliedMotion_5034 is missing required members");
}

} // namespace tmc51x0_test_config
