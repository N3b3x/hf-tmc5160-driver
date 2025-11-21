/**
 * @file sinusoidal.cpp
 * @brief Sinusoidal motion pattern example for TMC5160 stepper motor driver
 *
 * This example demonstrates sinusoidal motion control using the TMC5160 driver.
 * The motor moves in a sinusoidal pattern with configurable frequency,
 * amplitude, and number of rounds.
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - TMC5160 stepper motor driver
 * - Stepper motor connected to TMC5160
 * - SPI connection between ESP32 and TMC5160
 *
 * Pin Configuration (modify as needed):
 * - SPI: MOSI=23, MISO=19, SCLK=18, CS=5
 * - Control: EN=2, DIR=4, STEP=15
 *
 * Algorithm adapted from TMC5160-Sinusoidal-Movement archived driver.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 */

#include "../../../inc/tmc5160.hpp"
#include "esp32_tmc5160_bus.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char* TAG = "Sinusoidal";

/**
 * @brief Sinusoidal motion controller class
 *
 * Generates sinusoidal step timing based on frequency and amplitude parameters.
 */
class SinusoidalMotion {
private:
  tmc5160::TMC5160<Esp32SPI>* driver_;
  double frequency_;      // Frequency in Hz
  int amplitude_;         // Amplitude (affects step timing variation)
  uint32_t step_count_;   // Total steps per round
  int rounds_;            // Number of rounds to execute
  bool direction_;        // Movement direction
  uint32_t init_time_;    // Initial time reference
  uint32_t timestamp_;    // Current timestamp
  uint32_t current_step_; // Current step count
  bool initialized_;

public:
  SinusoidalMotion(tmc5160::TMC5160<Esp32SPI>* driver)
      : driver_(driver), frequency_(5.0), amplitude_(500), step_count_(55000), rounds_(2), direction_(true),
        init_time_(0), timestamp_(0), current_step_(0), initialized_(false) {}

  // Getters for logging
  double GetFrequency() const {
    return frequency_;
  }
  int GetAmplitude() const {
    return amplitude_;
  }
  uint32_t GetStepCount() const {
    return step_count_;
  }
  int GetRounds() const {
    return rounds_;
  }

  /**
   * @brief Configure sinusoidal motion parameters
   * @param freq Frequency in Hz
   * @param amp Amplitude (affects timing variation)
   * @param steps Number of steps per round
   * @param dir Direction (true=forward, false=backward)
   * @param rnds Number of rounds
   */
  void Config(double freq, int amp, uint32_t steps, bool dir, int rnds) {
    frequency_ = freq;
    amplitude_ = amp;
    step_count_ = steps;
    direction_ = dir;
    rounds_ = rnds;
    initialized_ = false;
  }

  /**
   * @brief Run sinusoidal motion (call repeatedly in loop)
   */
  void Run() {
    if (!initialized_) {
      initialized_ = true;
      init_time_ = esp_timer_get_time() / 1000; // Convert to milliseconds
      timestamp_ = init_time_;
      current_step_ = 0;
    }

    if (current_step_ >= step_count_) {
      if (rounds_ > 1) {
        current_step_ = 0;
        direction_ = !direction_;
        rounds_--;
      } else {
        initialized_ = false;
        return;
      }
    }

    // Calculate time delay based on sinusoidal function
    int time_delay = calculateTimeDelay();

    // Check if it's time to step
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - timestamp_ >= time_delay) {
      // Generate step pulse
      driver_->GetComm().GpioSetActive(tmc5160::TMC5160CtrlPin::STEP);
      driver_->GetComm().DelayUs(10); // Short pulse width
      driver_->GetComm().GpioSetInactive(tmc5160::TMC5160CtrlPin::STEP);

      timestamp_ = current_time;
      current_step_++;

      // Update direction pin
      driver_->GetComm().GpioSet(direction_ ? tmc5160::TMC5160CtrlPin::DIR : tmc5160::TMC5160CtrlPin::DIR,
                                 direction_ ? tmc5160::GpioSignal::ACTIVE : tmc5160::GpioSignal::INACTIVE);
    }
  }

private:
  /**
   * @brief Calculate time delay for next step based on sinusoidal function
   * @return Time delay in milliseconds
   */
  int calculateTimeDelay() {
    // Sinusoidal time calculation: amplitude * sin(frequency * time)
    // Convert frequency from Hz to rad/s and scale time
    const double radian_conversion = M_PI / 180.0;
    const double time_format_factor = 300.0; // Scaling factor

    uint32_t elapsed_time = (esp_timer_get_time() / 1000) - init_time_;
    double sin_value = sin(frequency_ * radian_conversion * elapsed_time / time_format_factor);

    int base_delay = 150; // Base delay in microseconds
    int variable_delay = static_cast<int>(amplitude_ * sin_value);

    return base_delay + variable_delay;
  }
};

extern "C" void app_main() {
  ESP_LOGI(TAG, "TMC5160 Sinusoidal Motion Example");

  // Create SPI communication interface
  Esp32SPI spi(SPI2_HOST,
               GPIO_NUM_23, // MOSI
               GPIO_NUM_19, // MISO
               GPIO_NUM_18, // SCLK
               GPIO_NUM_5,  // CS
               GPIO_NUM_2,  // EN
               GPIO_NUM_4,  // DIR
               GPIO_NUM_15, // STEP
               4000000);    // 4 MHz SPI clock

  // Initialize SPI interface
  if (!spi.Initialize()) {
    ESP_LOGE(TAG, "Failed to initialize SPI interface");
    return;
  }

  // Create TMC5160 driver instance
  tmc5160::TMC5160<Esp32SPI> driver(spi);

  // Configure driver
  tmc5160::DriverConfig cfg{};
  cfg.motor.irun = 20;
  cfg.motor.ihold = 10;
  cfg.chopper.mres = 5; // 32 microsteps for smooth motion
  cfg.chopper.intpol = true;

  // Initialize driver
  if (!driver.Initialize(cfg)) {
    ESP_LOGE(TAG, "Failed to initialize TMC5160 driver");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Configure for external step/dir mode (if needed)
  // For sinusoidal motion, we'll generate step pulses manually
  driver.motorControl.Enable();

  // Create sinusoidal motion controller
  SinusoidalMotion motion(&driver);

  // Configure sinusoidal motion
  // frequency: 5 Hz, amplitude: 500, steps: 55000, direction: forward, rounds:
  // 2
  motion.Config(5.0, 500, 55000, true, 2);

  ESP_LOGI(TAG,
           "Starting sinusoidal motion: freq=%.1f Hz, amplitude=%d, steps=%lu, "
           "rounds=%d",
           motion.GetFrequency(), motion.GetAmplitude(), motion.GetStepCount(), motion.GetRounds());

  // Run sinusoidal motion
  while (true) {
    motion.Run();
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent tight loop
  }
}
