/**
 * @file abn_encoder_reader.cpp
 * @brief ABN Encoder Position Reader - Continuously reads encoder position
 *
 * This example demonstrates how to configure and continuously read position from
 * an ABN (incremental) encoder connected to the TMC5160.
 *
 * Configured for AS5047U Encoder:
 * - 14-bit resolution: 16,384 positions per revolution
 * - ABI (incremental) mode: 4,096 PPR (pulses per revolution)
 * - Quadrature encoding: 4,096 PPR × 4 = 16,384 counts per revolution
 * - Index pulse (I channel): One pulse per revolution (equivalent to N channel)
 * - Output: A, B, and I (index) channels
 *
 * ABN Encoder Configuration:
 * - A/B channels: Quadrature encoder signals (required)
 * - N/I channel: Index/zero pulse (one pulse per revolution)
 * - AS5047U default: 4,096 PPR in ABI mode
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - AS5047U encoder connected to TMC5160 encoder pins:
 *   - A channel → TMC5160 ENC_A pin
 *   - B channel → TMC5160 ENC_B pin
 *   - I channel (index) → TMC5160 ENC_N pin
 * - SPI connection between ESP32 and TMC5160
 * - Motor connected (encoder reads motor position)
 *
 * Pin Configuration (uses default dev board pins from esp32_tmc51x0_test_config.hpp):
 * - SPI: MOSI=6, MISO=2, SCLK=5, CS=18
 * - Control: EN=11, CLK=10, DIAG0=23, DIAG1=15
 * - Encoder: A, B, I (index) channels connected to TMC5160 encoder interface
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cinttypes>

static const char* TAG = "ABN_Encoder";

//=============================================================================
// CONFIGURATION - Adjust these for your encoder
//=============================================================================

// Test rig selection (compile-time constant) - automatically selects motor, board, and platform
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;

//=============================================================================
// AS5047U Encoder Configuration
//=============================================================================
// AS5047U Specifications:
// - 14-bit resolution: 16,384 positions per revolution
// - ABI mode: 4,096 PPR (pulses per revolution)
// - Quadrature encoding: 4 counts per pulse = 16,384 counts/rev
// - Index pulse (I channel): One pulse per revolution (active high)
// - Operating voltage: 3.0V to 5.5V
// - Temperature range: -40°C to +150°C

// ABN Encoder Configuration
static constexpr uint16_t ENCODER_PPR = 4096;  // AS5047U ABI mode: 4,096 PPR
                                              // With quadrature: 4,096 × 4 = 16,384 counts/rev
                                              // This matches the 14-bit resolution (2^14 = 16,384)
static constexpr bool ENCODER_INVERT_DIRECTION = false;  // Set to true if encoder direction is reversed

// N Channel Configuration (Index/Zero Pulse - AS5047U I channel)
// AS5047U provides an index pulse (I channel) that pulses once per revolution
// This is equivalent to the N channel in ABN terminology
static constexpr tmc51x0::ReferenceSwitchActiveLevel N_CHANNEL_ACTIVE = 
    tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH;  // AS5047U index pulse is active high
static constexpr tmc51x0::EncoderNSensitivity N_SENSITIVITY = 
    tmc51x0::EncoderNSensitivity::RISING_EDGE;  // Trigger on rising edge of index pulse
static constexpr tmc51x0::EncoderClearMode N_CLEAR_MODE = 
    tmc51x0::EncoderClearMode::ONCE;  // Clear encoder position once on index pulse (useful for homing)

// Reading Configuration
static constexpr uint32_t READ_INTERVAL_MS = 100;  // Read position every 100ms

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "ABN Encoder Position Reader (AS5047U)");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Encoder: AS5047U (14-bit, ABI mode)");
    ESP_LOGI(TAG, "Encoder PPR: %u (quadrature: %u counts/rev)", ENCODER_PPR, ENCODER_PPR * 4);
    ESP_LOGI(TAG, "Resolution: 14-bit = 16,384 positions/rev");
    ESP_LOGI(TAG, "Invert Direction: %s", ENCODER_INVERT_DIRECTION ? "Yes" : "No");
    ESP_LOGI(TAG, "Index Channel (I/N): %s, Sensitivity: %d, Clear Mode: %d", 
             N_CHANNEL_ACTIVE == tmc51x0::ReferenceSwitchActiveLevel::ACTIVE_HIGH ? "HIGH" : "LOW",
             static_cast<int>(N_SENSITIVITY), static_cast<int>(N_CLEAR_MODE));
    ESP_LOGI(TAG, "Read Interval: %lu ms", READ_INTERVAL_MS);
    ESP_LOGI(TAG, "==========================================");

    // 1. Initialize SPI Bus
    ESP_LOGI(TAG, "Initializing SPI bus...");
    auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
    tmc51x0::PinActiveLevels active_levels;
    
    Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, tmc51x0_test_config::SPI_CLOCK_SPEED_HZ, active_levels);
    auto spi_init_result = spi.Initialize();
    if (!spi_init_result) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus (ErrorCode: %d)", static_cast<int>(spi_init_result.Error()));
        return;
    }
    ESP_LOGI(TAG, "✓ SPI bus initialized");

    // 2. Create and Initialize Driver
    ESP_LOGI(TAG, "Initializing TMC5160 driver...");
    tmc51x0::TMC51x0<Esp32SPI> driver(spi);
    
    // Configure driver from test rig (motor, board, platform)
    tmc51x0::DriverConfig cfg{};
    tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
    
    auto driver_init_result = driver.Initialize(cfg);
    if (!driver_init_result) {
        ESP_LOGE(TAG, "Failed to initialize TMC5160 driver (ErrorCode: %d - %s)", 
                 static_cast<int>(driver_init_result.Error()), driver_init_result.ErrorMessage());
        return;
    }
    ESP_LOGI(TAG, "✓ Driver initialized successfully");

    // 3. Configure ABN Encoder (AS5047U)
    ESP_LOGI(TAG, "Configuring AS5047U encoder (ABI mode)...");
    tmc51x0::EncoderConfig enc_cfg{};
    
    // Index Channel Configuration (AS5047U I channel → TMC5160 N channel)
    // AS5047U provides an index pulse (I channel) once per revolution
    enc_cfg.n_channel_active = N_CHANNEL_ACTIVE;  // ACTIVE_HIGH for AS5047U
    enc_cfg.n_sensitivity = N_SENSITIVITY;  // RISING_EDGE for AS5047U
    enc_cfg.clear_mode = N_CLEAR_MODE;  // ONCE: clear encoder position on index pulse
    
    // A/B Polarity (AS5047U ABI mode uses standard quadrature)
    enc_cfg.ignore_ab_polarity = true;  // Ignore A/B polarity for N channel validation
    
    // Additional encoder features
    enc_cfg.clear_enc_x_on_event = (N_CLEAR_MODE != tmc51x0::EncoderClearMode::DISABLED);  // Clear on index if enabled
    enc_cfg.latch_xactual_with_enc = (N_CLEAR_MODE != tmc51x0::EncoderClearMode::DISABLED);  // Latch motor position on index
    enc_cfg.prescaler_mode = tmc51x0::EncoderPrescalerMode::BINARY;  // Binary prescaler mode
    
    // Encoder resolution and deviation
    enc_cfg.pulses_per_rev = ENCODER_PPR;  // 4,096 PPR for AS5047U ABI mode
    enc_cfg.allowed_deviation_steps = 50;  // Allow 50 steps deviation before warning
    
    // Configure encoder
    auto enc_config_result = driver.encoder.Configure(enc_cfg);
    if (!enc_config_result) {
        ESP_LOGE(TAG, "Failed to configure encoder (ErrorCode: %d - %s)", 
                 static_cast<int>(enc_config_result.Error()), enc_config_result.ErrorMessage());
        return;
    }
    ESP_LOGI(TAG, "✓ Encoder configured");

    // 4. Set Encoder Resolution
    ESP_LOGI(TAG, "Setting encoder resolution...");
    constexpr uint16_t output_full_steps = 
        tmc51x0_test_config::GetTestRigMotorOutputFullSteps<SELECTED_TEST_RIG>();
    
    auto enc_res_result = driver.encoder.SetResolution(
        output_full_steps,  // Motor steps per revolution
        ENCODER_PPR,        // Encoder pulses per revolution
        ENCODER_INVERT_DIRECTION  // Invert direction if needed
    );
    
    if (!enc_res_result) {
        ESP_LOGW(TAG, "Encoder resolution set with approximation (ErrorCode: %d)", 
                 static_cast<int>(enc_res_result.Error()));
    } else {
        ESP_LOGI(TAG, "✓ Encoder resolution set: Motor=%u steps/rev, Encoder=%u PPR", 
                 output_full_steps, ENCODER_PPR);
    }

    // 5. Enable Motor (optional - encoder can be read even when motor is disabled)
    ESP_LOGI(TAG, "Enabling motor...");
    auto enable_result = driver.motorControl.Enable();
    if (!enable_result) {
        ESP_LOGE(TAG, "Failed to enable motor (ErrorCode: %d)", static_cast<int>(enable_result.Error()));
        return;
    }
    ESP_LOGI(TAG, "✓ Motor enabled");

    // 6. Verify encoder is working
    ESP_LOGI(TAG, "Verifying encoder connection...");
    auto test_pos_result = driver.encoder.GetPosition();
    if (!test_pos_result) {
        ESP_LOGE(TAG, "Failed to read encoder position - encoder may not be connected! (ErrorCode: %d)", 
                 static_cast<int>(test_pos_result.Error()));
        ESP_LOGE(TAG, "Please check:");
        ESP_LOGE(TAG, "  1. Encoder A/B channels are connected to TMC5160 ENC_A/ENC_B pins");
        ESP_LOGE(TAG, "  2. Encoder power supply is connected");
        ESP_LOGE(TAG, "  3. Encoder ground is connected");
        return;
    }
    ESP_LOGI(TAG, "✓ Encoder is working! Initial position: %" PRId32, test_pos_result.Value());

    // 7. Continuously read encoder position
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Starting continuous encoder position reading...");
    ESP_LOGI(TAG, "Press Ctrl+C to stop");
    ESP_LOGI(TAG, "==========================================");
    
    int32_t last_position = 0;
    uint32_t read_count = 0;
    
    while (true) {
        // Read encoder position
        auto pos_result = driver.encoder.GetPosition();
        
        if (pos_result) {
            int32_t current_position = pos_result.Value();
            int32_t position_change = current_position - last_position;
            
            // X_ENC register contains raw encoder quadrature counts
            // AS5047U ABI mode: 4,096 PPR × 4 (quadrature) = 16,384 counts/rev
            // Quadrature encoding: 4 counts per encoder pulse (A rising, A falling, B rising, B falling)
            // This matches the 14-bit resolution: 2^14 = 16,384 positions per revolution
            
            // Calculate position in degrees
            float position_deg = 0.0f;
            float position_revs = 0.0f;
            constexpr uint32_t counts_per_rev = ENCODER_PPR * 4;  // 16,384 for AS5047U
            if (counts_per_rev > 0) {
                // Convert quadrature counts to revolutions
                position_revs = static_cast<float>(current_position) / static_cast<float>(counts_per_rev);
                position_deg = position_revs * 360.0f;
            }
            
            // Also get motor position for comparison
            auto motor_pos_result = driver.rampControl.GetCurrentPosition(tmc51x0::Unit::Deg);
            float motor_pos_deg = motor_pos_result.IsOk() ? motor_pos_result.Value() : 0.0f;
            
            // Log position every read
            ESP_LOGI(TAG, "[%lu] Encoder: %" PRId32 " counts (%.4f rev, %.3f°), Motor: %.3f°, Change: %+" PRId32 " counts",
                     read_count, current_position, position_revs, position_deg, motor_pos_deg, position_change);
            
            // Check for deviation warning
            auto deviation_result = driver.encoder.IsDeviationDetected();
            if (deviation_result && deviation_result.Value()) {
                ESP_LOGW(TAG, "⚠ Encoder deviation detected! Motor and encoder positions differ significantly.");
                driver.encoder.ClearDeviationFlag();
            }
            
            // Check for latched position (if index channel is configured)
            // AS5047U index pulse (I channel) triggers once per revolution
            if (N_CLEAR_MODE != tmc51x0::EncoderClearMode::DISABLED) {
                auto latched_result = driver.encoder.GetLatchedPosition();
                if (latched_result) {
                    // Only log if latched position changed (index pulse detected)
                    static int32_t last_latched = 0;
                    if (latched_result.Value() != last_latched) {
                        float latched_revs = static_cast<float>(latched_result.Value()) / static_cast<float>(ENCODER_PPR * 4);
                        float latched_deg = latched_revs * 360.0f;
                        ESP_LOGI(TAG, "  → Index Pulse (I/N) Detected! Latched: %" PRId32 " counts (%.4f rev, %.3f°)",
                                 latched_result.Value(), latched_revs, latched_deg);
                        last_latched = latched_result.Value();
                    }
                }
            }
            
            last_position = current_position;
        } else {
            ESP_LOGW(TAG, "[%lu] Failed to read encoder position (ErrorCode: %d)", 
                     read_count, static_cast<int>(pos_result.Error()));
        }
        
        read_count++;
        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }
}

