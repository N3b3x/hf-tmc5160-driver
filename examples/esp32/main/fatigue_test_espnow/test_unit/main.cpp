/**
 * @file main.cpp
 * @brief Fatigue test unit with ESP-NOW communication
 * 
 * This is the test unit (receiver) that:
 * 1. Receives commands from UI board via ESP-NOW
 * 2. Performs bounds finding (stallguard or encoder-based)
 * 3. Runs fatigue test with sinusoidal motion
 * 4. Sends status updates back to UI board
 * 
 * Supports both StallGuard2 and encoder-based bounds detection.
 */

#include "../../../inc/tmc51x0.hpp"
#include "test_config/esp32_tmc51x0_bus.hpp"
#include "test_config/esp32_tmc51x0_test_config.hpp"

#include "../espnow_protocol.hpp"
#include "espnow_receiver.hpp"
#include "bounds_finder.hpp"
#include <memory>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <cstdarg>
#include <map>

static const char* TAG = "FatigueTestUnit";

// Test rig selection
static constexpr tmc51x0_test_config::TestRigType SELECTED_TEST_RIG = 
    tmc51x0_test_config::TestRigType::TEST_RIG_FATIGUE;

// Forward declarations
static void espnow_command_task(void* arg);
static void motion_control_task(void* arg);
static void status_update_task(void* arg);

// Global state (forward declarations)
static tmc51x0::TMC51x0<Esp32SPI>* g_driver = nullptr;
static Settings g_settings{};
static QueueHandle_t g_espnowQueue = nullptr;
static bool g_bounds_found = false;
static bool g_use_stallguard = true;

// g_motion will be declared after FatigueTestMotion is defined

// RAII Mutex classes (same as fatigue_test_encoder.cpp)
class Esp32TmcMutex {
public:
    Esp32TmcMutex() noexcept : handle_(xSemaphoreCreateMutex()) {
        if (handle_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create mutex");
        }
    }
    ~Esp32TmcMutex() noexcept {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
            handle_ = nullptr;
        }
    }
    Esp32TmcMutex(const Esp32TmcMutex&) = delete;
    Esp32TmcMutex& operator=(const Esp32TmcMutex&) = delete;
    SemaphoreHandle_t native_handle() const noexcept { return handle_; }
    bool is_valid() const noexcept { return handle_ != nullptr; }
private:
    SemaphoreHandle_t handle_;
};

class TmcMutexGuard {
public:
    explicit TmcMutexGuard(Esp32TmcMutex& mutex) noexcept : mutex_(mutex), locked_(false) {
        if (mutex_.is_valid()) {
            SemaphoreHandle_t handle = mutex_.native_handle();
            if (xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE) {
                locked_ = true;
            }
        }
    }
    ~TmcMutexGuard() noexcept { unlock(); }
    void unlock() noexcept {
        if (locked_ && mutex_.is_valid()) {
            SemaphoreHandle_t handle = mutex_.native_handle();
            xSemaphoreGive(handle);
            locked_ = false;
        }
    }
    bool is_locked() const noexcept { return locked_; }
private:
    Esp32TmcMutex& mutex_;
    bool locked_;
};

// Include full FatigueTestMotion implementation (after mutex classes are defined)
#include "fatigue_motion.hpp"
#include "fatigue_motion_impl.hpp"

// Now declare g_motion after FatigueTestMotion is fully defined
static FatigueTest::FatigueTestMotion* g_motion = nullptr;

// ESP-NOW command handler task
static void espnow_command_task(void* arg)
{
    const char* task_name = "espnow_cmd";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = 5000000; // Log every 5 seconds
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    
    ProtoEvent ev{};
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        // Use timeout so we can check time even when no messages arrive
        if (xQueueReceive(g_espnowQueue, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (ev.type) {
                case ProtoEventType::CONFIG_REQUEST:
                    ESP_LOGI(TAG, "Config request received");
                    EspNowReceiver::send_config_response(g_settings);
                    break;
                    
                case ProtoEventType::CONFIG_SET:
                    ESP_LOGI(TAG, "Config set: cycles=%u, tper=%u, dwell=%u, bounds=%s",
                             ev.data.config.cycle_amount,
                             ev.data.config.time_per_cycle,
                             ev.data.config.dwell_time,
                             ev.data.config.bounds_method_stallguard ? "StallGuard" : "Encoder");
                    g_settings = ev.data.config;
                    g_use_stallguard = ev.data.config.bounds_method_stallguard;
                    
                    if (g_motion) {
                        g_motion->SetTargetCycles(g_settings.cycle_amount);
                        g_motion->SetDwellTimes(g_settings.dwell_time * 1000, g_settings.dwell_time * 1000);
                        // Convert time_per_cycle to frequency
                        float freq = 1.0f / (float)g_settings.time_per_cycle;
                        g_motion->SetFrequency(freq);
                    }
                    EspNowReceiver::send_config_ack(true, 0);
                    break;
                    
                case ProtoEventType::START:
                    ESP_LOGI(TAG, "Start command received");
                    if (!g_bounds_found) {
                        ESP_LOGW(TAG, "Bounds not found yet, cannot start");
                        EspNowReceiver::send_start_ack();
                        EspNowReceiver::send_error(1, 0); // Error: bounds not found
                        break;
                    }
                    if (g_motion && g_motion->Start()) {
                        EspNowReceiver::send_start_ack();
                        EspNowReceiver::send_status_update(0, TestState::RUNNING);
                    } else {
                        EspNowReceiver::send_start_ack();
                        EspNowReceiver::send_error(2, 0); // Error: start failed
                    }
                    break;
                    
                case ProtoEventType::PAUSE:
                    ESP_LOGI(TAG, "Pause command received");
                    if (g_motion) {
                        g_motion->Stop();
                        EspNowReceiver::send_pause_ack();
                        EspNowReceiver::send_status_update(g_motion->GetCurrentCycles(), TestState::PAUSED);
                    }
                    break;
                    
                case ProtoEventType::RESUME:
                    ESP_LOGI(TAG, "Resume command received");
                    if (g_motion && g_motion->Start()) {
                        EspNowReceiver::send_resume_ack();
                        EspNowReceiver::send_status_update(g_motion->GetCurrentCycles(), TestState::RUNNING);
                    }
                    break;
                    
                case ProtoEventType::STOP:
                    ESP_LOGI(TAG, "Stop command received");
                    if (g_motion) {
                        uint32_t cycles = g_motion->GetCurrentCycles();
                        g_motion->Stop();
                        EspNowReceiver::send_stop_ack();
                        EspNowReceiver::send_status_update(cycles, TestState::IDLE);
                    }
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unhandled event type: %d", (int)ev.type);
                    break;
            }
        }
    }
}

// Motion control task
static void motion_control_task(void* arg)
{
    const char* task_name = "motion_ctrl";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = 5000000; // Log every 5 seconds
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        if (g_motion) {
            g_motion->Update();
            
            // Check if test completed
            if (!g_motion->IsRunning()) {
                auto status = g_motion->GetStatus();
                if (status.current_cycles >= status.target_cycles && status.target_cycles > 0) {
                    EspNowReceiver::send_status_update(status.current_cycles, TestState::COMPLETED);
                    EspNowReceiver::send_test_complete();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms update rate
    }
}

// Status update task (sends periodic status to UI board)
static void status_update_task(void* arg)
{
    const char* task_name = "status_upd";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = 5000000; // Log every 5 seconds
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
        
        if (g_motion && g_motion->IsRunning()) {
            auto status = g_motion->GetStatus();
            TestState state = status.running ? TestState::RUNNING : TestState::IDLE;
            EspNowReceiver::send_status_update(status.current_cycles, state);
        }
    }
}

//=============================================================================
// UART Command Parser - Redesigned Architecture
//=============================================================================

// Forward declarations for command handlers
struct ParsedCommand;
namespace FatigueTest { class FatigueTestMotion; }
static bool HandleSet(const ParsedCommand& cmd, FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleStart(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleStop(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandlePause(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleResume(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleReset(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleStatus(FatigueTest::FatigueTestMotion& motion) noexcept;
static bool HandleHelp(const std::string& topic) noexcept;

/**
 * @brief Command types for word-based commands
 */
enum class CommandType {
    SET,      // set <options>
    START,    // start
    STOP,     // stop
    PAUSE,    // pause
    RESUME,   // resume
    RESET,    // reset
    STATUS,   // status
    HELP      // help [command]
};

/**
 * @brief Option types for SET command
 */
enum class OptionType {
    FREQUENCY,  // -f, --frequency
    DWELL,      // -d, --dwell
    BOUNDS,     // -b, --bounds
    CYCLES      // -c, --cycles
};

/**
 * @brief Parsed command structure
 */
struct ParsedCommand {
    CommandType type;
    std::vector<std::pair<OptionType, std::vector<std::string>>> options; // For SET command
    std::string help_topic; // For HELP command
};

/**
 * @brief Visual output formatting system
 */
namespace CommandOutput {
    static constexpr int BOX_WIDTH = 78;
    
    void PrintSuccess(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGI(TAG, "✓ %s", buffer);
    }
    
    void PrintError(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGE(TAG, "✗ Error: %s", buffer);
    }
    
    void PrintInfo(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGI(TAG, "ℹ %s", buffer);
    }
    
    void PrintWarning(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        ESP_LOGW(TAG, "⚠ Warning: %s", buffer);
    }
    
    void PrintHeader(const char* title) {
        ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
        char header[BOX_WIDTH + 1];
        int title_len = strlen(title);
        int padding = (BOX_WIDTH - title_len - 2) / 2;
        snprintf(header, sizeof(header), "║%*s%s%*s║", padding, "", title, BOX_WIDTH - padding - title_len - 2, "");
        ESP_LOGI(TAG, "%s", header);
        ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════════════╣");
    }
    
    void PrintSeparator() {
        ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════════════╣");
    }
    
    void PrintFooter() {
        ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    }
    
    void PrintTableRow(const char* label, const char* value) {
        // Use a larger buffer to account for multi-byte Unicode characters
        char row[BOX_WIDTH * 2 + 1];
        // Truncate value if needed to prevent format truncation warning
        char value_trunc[43];
        snprintf(value_trunc, sizeof(value_trunc), "%.42s", value);
        // Format: "║  " + label (30) + " " + value (42) + "║"
        int written = snprintf(row, sizeof(row), "║  %-30s %-42s║", label, value_trunc);
        if (written < 0 || written >= static_cast<int>(sizeof(row))) {
            // Truncation occurred, ensure null termination
            row[sizeof(row) - 1] = '\0';
        }
        ESP_LOGI(TAG, "%s", row);
    }
    
    void PrintEmptyLine() {
        ESP_LOGI(TAG, "║%*s║", BOX_WIDTH - 2, "");
    }
    
    void PrintCommandResult(const char* command, bool success, const char* details = nullptr) {
        if (success) {
            if (details) {
                PrintSuccess("%s: %s", command, details);
            } else {
                PrintSuccess("%s completed successfully", command);
            }
        } else {
            if (details) {
                PrintError("%s failed: %s", command, details);
            } else {
                PrintError("%s failed", command);
            }
        }
    }
}

/**
 * @brief Option parser for SET command
 */
class OptionParser {
public:
    struct OptionDef {
        OptionType type;
        const char* short_name;
        const char* long_name;
        const char* description;
        int min_args;
        int max_args;
    };
    
    static const OptionDef* FindOption(const std::string& name) {
        static const OptionDef options[] = {
            {OptionType::FREQUENCY, "-f", "--frequency", "Motion frequency in Hz", 1, 1},
            {OptionType::DWELL, "-d", "--dwell", "Dwell times in ms (min, max)", 2, 2},
            {OptionType::BOUNDS, "-b", "--bounds", "Angle bounds in degrees (min, max)", 2, 2},
            {OptionType::CYCLES, "-c", "--cycles", "Target cycle count (0 = infinite)", 1, 1}
        };
        
        for (const auto& opt : options) {
            if (name == opt.short_name || name == opt.long_name) {
                return &opt;
            }
        }
        return nullptr;
    }
    
    static bool ParseOptions(const std::vector<std::string>& tokens, 
                            std::vector<std::pair<OptionType, std::vector<std::string>>>& options) {
        options.clear();
        
        for (size_t i = 0; i < tokens.size(); ++i) {
            const OptionDef* def = FindOption(tokens[i]);
            if (def) {
                // Flag-based option (e.g., "-f", "--frequency")
                // Check if we have enough arguments
                if (i + def->min_args >= tokens.size()) {
                    return false; // Not enough arguments
                }
                
                // Extract arguments
                std::vector<std::string> args;
                int arg_count = std::min(def->max_args, static_cast<int>(tokens.size() - i - 1));
                for (int j = 0; j < arg_count; ++j) {
                    args.push_back(tokens[i + j + 1]);
                }
                
                options.push_back({def->type, args});
                i += arg_count; // Skip the arguments we just consumed
            } else {
                // Check if it's a word-based option (e.g., "frequency", "dwell")
                if (tokens[i] == "frequency" && i + 1 < tokens.size()) {
                    options.push_back({OptionType::FREQUENCY, {tokens[i + 1]}});
                    i++;
                } else if (tokens[i] == "dwell" && i + 2 < tokens.size()) {
                    options.push_back({OptionType::DWELL, {tokens[i + 1], tokens[i + 2]}});
                    i += 2;
                } else if (tokens[i] == "bounds" && i + 2 < tokens.size()) {
                    options.push_back({OptionType::BOUNDS, {tokens[i + 1], tokens[i + 2]}});
                    i += 2;
                } else if (tokens[i] == "cycles" && i + 1 < tokens.size()) {
                    options.push_back({OptionType::CYCLES, {tokens[i + 1]}});
                    i++;
                } else {
                    // Unknown token - could be a value from previous option or invalid
                    // Check if previous option consumed all its args - if so, this is invalid
                    return false; // Unknown option or unexpected token
                }
            }
        }
        
        return true;
    }
    
    static const OptionDef* GetOptionDef(OptionType type) {
        static const OptionDef options[] = {
            {OptionType::FREQUENCY, "-f", "--frequency", "Motion frequency in Hz", 1, 1},
            {OptionType::DWELL, "-d", "--dwell", "Dwell times in ms (min, max)", 2, 2},
            {OptionType::BOUNDS, "-b", "--bounds", "Angle bounds in degrees (min, max)", 2, 2},
            {OptionType::CYCLES, "-c", "--cycles", "Target cycle count (0 = infinite)", 1, 1}
        };
        
        for (const auto& opt : options) {
            if (opt.type == type) {
                return &opt;
            }
        }
        return nullptr;
    }
};

/**
 * @brief Redesigned UART command parser with word-based commands
 */
class UartCommandParser {
private:
    uart_port_t uart_port_;
    char rx_buffer_[256];
    char line_buffer_[256];
    size_t line_buffer_pos_;
    static constexpr size_t RX_BUF_SIZE = 256;

    /**
     * @brief Parse command line into tokens
     */
    std::vector<std::string> Tokenize(const char* line) noexcept {
        std::vector<std::string> tokens;
        std::string current;
        bool in_quotes = false;

        for (const char* p = line; *p != '\0'; ++p) {
            if (*p == '"') {
                in_quotes = !in_quotes;
            } else if (isspace(*p) && !in_quotes) {
                if (!current.empty()) {
                    tokens.push_back(current);
                    current.clear();
                }
            } else {
                current += *p;
            }
        }

        if (!current.empty()) {
            tokens.push_back(current);
        }

        return tokens;
    }
    
    /**
     * @brief Parse command type from token
     */
    CommandType ParseCommandType(const std::string& token) noexcept {
        if (token == "set") return CommandType::SET;
        if (token == "start") return CommandType::START;
        if (token == "stop") return CommandType::STOP;
        if (token == "pause") return CommandType::PAUSE;
        if (token == "resume") return CommandType::RESUME;
        if (token == "reset") return CommandType::RESET;
        if (token == "status") return CommandType::STATUS;
        if (token == "help") return CommandType::HELP;
        return CommandType::HELP; // Default to help for unknown
    }
    
    /**
     * @brief Parse command line into ParsedCommand structure
     */
    bool ParseCommand(const std::vector<std::string>& tokens, ParsedCommand& cmd) noexcept {
        if (tokens.empty()) {
            return false;
        }
        
        cmd.type = ParseCommandType(tokens[0]);
        
        if (cmd.type == CommandType::SET) {
            // Parse options for SET command
            std::vector<std::string> option_tokens(tokens.begin() + 1, tokens.end());
            if (option_tokens.empty()) {
                // Empty SET command - will be handled by HandleSet
                return true;
            }
            if (!OptionParser::ParseOptions(option_tokens, cmd.options)) {
                CommandOutput::PrintError("Failed to parse SET command options");
                CommandOutput::PrintInfo("Use 'help set' for usage information");
                return false;
            }
        } else if (cmd.type == CommandType::HELP) {
            // Parse help topic if provided
            if (tokens.size() > 1) {
                cmd.help_topic = tokens[1];
            }
        }
        
        return true;
    }

public:
    UartCommandParser(uart_port_t uart_port) : uart_port_(uart_port), line_buffer_pos_(0) {
        line_buffer_[0] = '\0';
        
        // On ESP32C6, UART_NUM_0 is typically used by the console
        // Try to install the driver - if it's already installed, we'll get ESP_ERR_INVALID_STATE
        ESP_LOGI(TAG, "Configuring UART driver on port %d", uart_port_);
        
        // Configure UART
        uart_config_t uart_config = {};
        uart_config.baud_rate = 115200;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_DEFAULT;

        // Try to install UART driver with RX buffer
        // If driver is already installed (by console), we'll get ESP_ERR_INVALID_STATE
        esp_err_t ret = uart_driver_install(uart_port_, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
        if (ret == ESP_ERR_INVALID_STATE) {
            // Driver already installed (likely by console) - that's fine, we can use it
            ESP_LOGI(TAG, "UART port %d already has driver installed (using existing)", uart_port_);
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
            return;
        } else {
            ESP_LOGI(TAG, "UART driver installed on port %d", uart_port_);
        }
        
        // Configure UART parameters (safe to call even if driver was already installed)
        ret = uart_param_config(uart_port_, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
            return;
        }
        
        // For USB serial (UART_NUM_0), pins are typically handled by USB driver
        ret = uart_set_pin(uart_port_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
            return;
        }
        
        // Verify we can read from the UART
        size_t test_available = 0;
        ret = uart_get_buffered_data_len(uart_port_, &test_available);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "UART port %d ready, initial buffered data: %zu bytes", 
                     uart_port_, test_available);
        } else {
            ESP_LOGE(TAG, "UART port %d not accessible: %s", uart_port_, esp_err_to_name(ret));
        }
    }

    /**
     * @brief Process a command line
     */
    bool ProcessCommand(const char* line, FatigueTest::FatigueTestMotion& motion) noexcept {
        if (!line || strlen(line) == 0) {
            return false;
        }

        std::vector<std::string> tokens = Tokenize(line);
        if (tokens.empty()) {
            return false;
        }
        
        ParsedCommand cmd;
        if (!ParseCommand(tokens, cmd)) {
            CommandOutput::PrintError("Failed to parse command");
            return false;
        }
        
        // Route to appropriate handler
        switch (cmd.type) {
            case CommandType::SET:
                return HandleSet(cmd, motion);
            case CommandType::START:
                return HandleStart(motion);
            case CommandType::STOP:
                return HandleStop(motion);
            case CommandType::PAUSE:
                return HandlePause(motion);
            case CommandType::RESUME:
                return HandleResume(motion);
            case CommandType::RESET:
                return HandleReset(motion);
            case CommandType::STATUS:
                return HandleStatus(motion);
            case CommandType::HELP:
                return HandleHelp(cmd.help_topic);
            default:
                CommandOutput::PrintError("Unknown command type");
                return false;
        }
    }

    /**
     * @brief Read and process commands from UART (line-by-line)
     */
    void ProcessUartCommands(FatigueTest::FatigueTestMotion& motion) noexcept {
        // Check available bytes first
        size_t available = 0;
        esp_err_t ret = uart_get_buffered_data_len(uart_port_, &available);
        if (ret != ESP_OK) {
            // Log error only occasionally to avoid spam
            static uint32_t error_count = 0;
            if (error_count++ % 100 == 0) {
                ESP_LOGE(TAG, "Failed to get UART buffered data length: %s (count=%lu)", 
                        esp_err_to_name(ret), error_count);
            }
            return;
        }
        
        // Log when data becomes available (for debugging)
        static size_t last_available = 0;
        if (available > 0 && available != last_available) {
            ESP_LOGI(TAG, "UART has %zu bytes available (was %zu)", available, last_available);
        }
        last_available = available;
        
        // Read available bytes from UART (non-blocking)
        // Use a small timeout to ensure we get complete data
        int len = uart_read_bytes(uart_port_, (uint8_t*)rx_buffer_, RX_BUF_SIZE - 1, 
                                  available > 0 ? pdMS_TO_TICKS(10) : 0);
        if (len > 0) {
            rx_buffer_[len] = '\0';
            ESP_LOGI(TAG, "UART read %d bytes: '%.*s'", len, len, rx_buffer_);
            
            // Process each character, accumulating into line buffer until newline
            for (int i = 0; i < len; i++) {
                char c = rx_buffer_[i];
                
                // Handle newline/carriage return - process the complete line
                if (c == '\n' || c == '\r') {
                    if (line_buffer_pos_ > 0) {
                        line_buffer_[line_buffer_pos_] = '\0';
                        ESP_LOGI(TAG, "Received UART command: '%s'", line_buffer_);
                        ProcessCommand(line_buffer_, motion);
                        line_buffer_pos_ = 0;
                        line_buffer_[0] = '\0';
                    }
                    // Skip multiple consecutive newlines
                    continue;
                }
                
                // Add character to line buffer (ignore if buffer is full)
                if (line_buffer_pos_ < (RX_BUF_SIZE - 1)) {
                    line_buffer_[line_buffer_pos_++] = c;
                } else {
                    // Buffer overflow - reset and log error
                    ESP_LOGW(TAG, "UART command line too long, resetting buffer");
                    line_buffer_pos_ = 0;
                    line_buffer_[0] = '\0';
                }
            }
        } else if (available > 0 && len == 0) {
            // Data is available but read returned 0 - this shouldn't happen
            ESP_LOGW(TAG, "UART has %zu bytes available but read returned 0", available);
        }
    }

};

//=============================================================================
// Command Handlers
//=============================================================================

/**
 * @brief Handle SET command with multiple options
 */
static bool HandleSet(const ParsedCommand& cmd, FatigueTest::FatigueTestMotion& motion) noexcept {
    if (cmd.options.empty()) {
        CommandOutput::PrintError("SET command requires at least one option");
        CommandOutput::PrintInfo("Use 'help set' for usage information");
        return false;
    }
    
    bool all_success = true;
    int success_count = 0;
    int failure_count = 0;
    
    // Process each option
    for (const auto& opt_pair : cmd.options) {
        OptionType opt_type = opt_pair.first;
        const std::vector<std::string>& args = opt_pair.second;
        bool success = false;
        
        switch (opt_type) {
            case OptionType::FREQUENCY: {
                if (args.empty()) {
                    CommandOutput::PrintError("Frequency option requires a value");
                    failure_count++;
                    break;
                }
                float freq = std::strtof(args[0].c_str(), nullptr);
                if (freq < 0.01f || freq > 10.0f) {
                    CommandOutput::PrintError("Frequency must be between 0.01 and 10.0 Hz (got %.2f)", freq);
                    failure_count++;
                    break;
                }
                success = motion.SetFrequency(freq);
                if (success) {
                    CommandOutput::PrintSuccess("Frequency set to %.2f Hz", freq);
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set frequency");
                    failure_count++;
                }
                break;
            }
            
            case OptionType::DWELL: {
                if (args.size() < 2) {
                    CommandOutput::PrintError("Dwell option requires 2 values (min_ms, max_ms)");
                    failure_count++;
                    break;
                }
                uint32_t min_ms = std::strtoul(args[0].c_str(), nullptr, 10);
                uint32_t max_ms = std::strtoul(args[1].c_str(), nullptr, 10);
                if (min_ms > 60000 || max_ms > 60000) {
                    CommandOutput::PrintError("Dwell times must be between 0 and 60000 ms");
                    failure_count++;
                    break;
                }
                if (min_ms > max_ms) {
                    CommandOutput::PrintError("Dwell min (%lu) must be <= max (%lu)", min_ms, max_ms);
                    failure_count++;
                    break;
                }
                success = motion.SetDwellTimes(min_ms, max_ms);
                if (success) {
                    CommandOutput::PrintSuccess("Dwell times set: %lu-%lu ms", min_ms, max_ms);
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set dwell times");
                    failure_count++;
                }
                break;
            }
            
            case OptionType::BOUNDS: {
                if (args.size() < 2) {
                    CommandOutput::PrintError("Bounds option requires 2 values (min_deg, max_deg)");
                    failure_count++;
                    break;
                }
                float min_deg = std::strtof(args[0].c_str(), nullptr);
                float max_deg = std::strtof(args[1].c_str(), nullptr);
                if (min_deg < -180.0f || max_deg > 180.0f) {
                    CommandOutput::PrintError("Bounds must be between -180° and +180°");
                    failure_count++;
                    break;
                }
                if (min_deg >= max_deg) {
                    CommandOutput::PrintError("Bounds min (%.2f°) must be < max (%.2f°)", min_deg, max_deg);
                    failure_count++;
                    break;
                }
                success = motion.SetLocalBoundsFromCenterDegrees(min_deg, max_deg);
                if (success) {
                    CommandOutput::PrintSuccess("Bounds set: %.2f° to %.2f° from center", min_deg, max_deg);
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set bounds");
                    failure_count++;
                }
                break;
            }
            
            case OptionType::CYCLES: {
                if (args.empty()) {
                    CommandOutput::PrintError("Cycles option requires a value");
                    failure_count++;
                    break;
                }
                uint32_t cycles = std::strtoul(args[0].c_str(), nullptr, 10);
                success = motion.SetTargetCycles(cycles);
                if (success) {
                    g_settings.cycle_amount = cycles;
                    if (cycles == 0) {
                        CommandOutput::PrintSuccess("Target cycles set to infinite");
                    } else {
                        CommandOutput::PrintSuccess("Target cycles set to %lu", cycles);
                    }
                    success_count++;
                } else {
                    CommandOutput::PrintError("Failed to set target cycles");
                    failure_count++;
                }
                break;
            }
            
            default:
                CommandOutput::PrintError("Unknown option type");
                failure_count++;
                break;
        }
    }
    
    // Summary
    if (failure_count == 0) {
        CommandOutput::PrintSuccess("All settings applied successfully (%d option(s))", success_count);
        return true;
    } else if (success_count > 0) {
        CommandOutput::PrintWarning("Some settings were not applied (%d succeeded, %d failed)", 
                                    success_count, failure_count);
        return false;
    } else {
        CommandOutput::PrintError("No settings were applied");
        return false;
    }
}

/**
 * @brief Handle START command
 */
static bool HandleStart(FatigueTest::FatigueTestMotion& motion) noexcept {
    if (!g_bounds_found) {
        CommandOutput::PrintError("Cannot start: bounds not found. Run bounds finding first.");
        return false;
    }
    
    auto status = motion.GetStatus();
    if (status.running) {
        CommandOutput::PrintWarning("Test is already running");
        return false;
    }
    
    if (motion.IsCycleComplete()) {
        CommandOutput::PrintError("Cycle count reached. Use 'reset' to reset cycles or set new target.");
        return false;
    }
    
    bool result = motion.Start();
    if (result) {
        CommandOutput::PrintSuccess("Fatigue test started");
        EspNowReceiver::send_start_ack();
        EspNowReceiver::send_status_update(motion.GetCurrentCycles(), TestState::RUNNING);
    } else {
        CommandOutput::PrintError("Failed to start fatigue test");
    }
    return result;
}

/**
 * @brief Handle STOP command
 */
static bool HandleStop(FatigueTest::FatigueTestMotion& motion) noexcept {
    auto status = motion.GetStatus();
    if (!status.running) {
        CommandOutput::PrintWarning("Test is not running");
        return false;
    }
    
    motion.Stop();
    CommandOutput::PrintSuccess("Fatigue test stopped");
    EspNowReceiver::send_stop_ack();
    EspNowReceiver::send_status_update(motion.GetCurrentCycles(), TestState::IDLE);
    return true;
}

/**
 * @brief Handle PAUSE command (future)
 */
static bool HandlePause(FatigueTest::FatigueTestMotion& motion) noexcept {
    (void)motion;
    CommandOutput::PrintInfo("Pause functionality not yet implemented");
    return false;
}

/**
 * @brief Handle RESUME command (future)
 */
static bool HandleResume(FatigueTest::FatigueTestMotion& motion) noexcept {
    (void)motion;
    CommandOutput::PrintInfo("Resume functionality not yet implemented");
    return false;
}

/**
 * @brief Handle RESET command
 */
static bool HandleReset(FatigueTest::FatigueTestMotion& motion) noexcept {
    motion.ResetCycles();
    CommandOutput::PrintSuccess("Cycle counter reset to 0");
    return true;
}

/**
 * @brief Handle STATUS command
 */
static bool HandleStatus(FatigueTest::FatigueTestMotion& motion) noexcept {
    FatigueTest::FatigueTestMotion::Status status = motion.GetStatus();
    
    CommandOutput::PrintHeader("MOTION STATUS");
    CommandOutput::PrintEmptyLine();
    
    char state_str[32];
    snprintf(state_str, sizeof(state_str), "%s", status.running ? "RUNNING" : "IDLE");
    CommandOutput::PrintTableRow("State:", state_str);
    
    char bounded_str[32];
    snprintf(bounded_str, sizeof(bounded_str), "%s", status.bounded ? "YES" : "NO");
    CommandOutput::PrintTableRow("Bounded:", bounded_str);
    
    char freq_str[64];
    snprintf(freq_str, sizeof(freq_str), "%.2f Hz (Estimated: %.2f Hz)", 
             status.frequency_hz, motion.GetEstimatedFrequency());
    CommandOutput::PrintTableRow("Frequency:", freq_str);
    
    char local_bounds_str[64];
    snprintf(local_bounds_str, sizeof(local_bounds_str), "%.2f° to %.2f° from center",
             status.min_degrees_from_center, status.max_degrees_from_center);
    CommandOutput::PrintTableRow("Local Bounds:", local_bounds_str);
    
    char global_bounds_str[64];
    snprintf(global_bounds_str, sizeof(global_bounds_str), "%.2f° to %.2f° from center",
             status.global_min_degrees, status.global_max_degrees);
    CommandOutput::PrintTableRow("Global Bounds:", global_bounds_str);
    
    char cycles_str[64];
    if (status.target_cycles == 0) {
        snprintf(cycles_str, sizeof(cycles_str), "%lu / infinite", status.current_cycles);
    } else {
        snprintf(cycles_str, sizeof(cycles_str), "%lu / %lu", 
                 status.current_cycles, status.target_cycles);
    }
    CommandOutput::PrintTableRow("Cycles:", cycles_str);
    
    char dwell_str[64];
    snprintf(dwell_str, sizeof(dwell_str), "%lu-%lu ms", status.dwell_min_ms, status.dwell_max_ms);
    CommandOutput::PrintTableRow("Dwell Times:", dwell_str);
    
    CommandOutput::PrintEmptyLine();
    CommandOutput::PrintFooter();
    
    return true;
}

/**
 * @brief Handle HELP command
 */
static bool HandleHelp(const std::string& topic) noexcept {
    if (topic.empty()) {
        // General help
        CommandOutput::PrintHeader("UART COMMAND INTERFACE");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintTableRow("set [OPTIONS...]", "Configure test parameters");
        CommandOutput::PrintTableRow("start", "Start fatigue test");
        CommandOutput::PrintTableRow("stop", "Stop fatigue test");
        CommandOutput::PrintTableRow("reset", "Reset cycle counter");
        CommandOutput::PrintTableRow("status", "Show current status");
        CommandOutput::PrintTableRow("help [command]", "Show help (general or specific)");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("Use 'help <command>' for detailed help on a specific command");
        CommandOutput::PrintFooter();
        return true;
    }
    
    // Specific command help
    if (topic == "set") {
        CommandOutput::PrintHeader("COMMAND HELP: set");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Configure test parameters. Multiple options can be set in one command.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  set [OPTIONS...]");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("OPTIONS:");
        CommandOutput::PrintInfo("  -f, --frequency <Hz>        Motion frequency (0.01 - 10.0 Hz)");
        CommandOutput::PrintInfo("  -d, --dwell <min> <max>     Dwell times in ms (0 - 60000)");
        CommandOutput::PrintInfo("  -b, --bounds <min> <max>    Angle bounds in degrees (-180 to 180)");
        CommandOutput::PrintInfo("  -c, --cycles <count>        Target cycles (0 = infinite)");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  set frequency 0.5");
        CommandOutput::PrintInfo("  set -f 0.5 -d 500 1000");
        CommandOutput::PrintInfo("  set -f 0.5 -d 500 1000 -b -60 60 -c 1000");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "start") {
        CommandOutput::PrintHeader("COMMAND HELP: start");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Start the fatigue test. Bounds must be found before starting.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  start");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  start");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "stop") {
        CommandOutput::PrintHeader("COMMAND HELP: stop");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Stop the currently running fatigue test.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  stop");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  stop");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "reset") {
        CommandOutput::PrintHeader("COMMAND HELP: reset");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Reset the cycle counter to 0.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  reset");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  reset");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else if (topic == "status") {
        CommandOutput::PrintHeader("COMMAND HELP: status");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("DESCRIPTION:");
        CommandOutput::PrintInfo("  Display current test status including all parameters.");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("SYNTAX:");
        CommandOutput::PrintInfo("  status");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintInfo("EXAMPLES:");
        CommandOutput::PrintInfo("  status");
        CommandOutput::PrintEmptyLine();
        CommandOutput::PrintFooter();
        return true;
    } else {
        CommandOutput::PrintError("Unknown help topic: %s", topic.c_str());
        CommandOutput::PrintInfo("Available help topics: set, start, stop, reset, status");
        return false;
    }
}

// UART Command Task
static void uart_command_task(void* arg)
{
    UartCommandParser* parser = static_cast<UartCommandParser*>(arg);
    const char* task_name = "uart_cmd";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = 5000000; // Log every 5 seconds
    
    ESP_LOGI(TAG, "[%s] Task started", task_name);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        if (g_motion && parser) {
            parser->ProcessUartCommands(*g_motion);
        } else {
            if (!g_motion) {
                ESP_LOGW(TAG, "[%s] g_motion is null, skipping UART command processing", task_name);
            }
            if (!parser) {
                ESP_LOGE(TAG, "[%s] parser is null!", task_name);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Check for commands every 50ms
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         Fatigue Test Unit with ESP-NOW Communication                         ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    // Initialize ESP-NOW receiver
    g_espnowQueue = xQueueCreate(10, sizeof(ProtoEvent));
    if (!EspNowReceiver::init(g_espnowQueue)) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW receiver");
        return;
    }

    // Get default pin configuration
    auto pin_config = tmc51x0_test_config::GetDefaultPinConfig();
    tmc51x0::PinActiveLevels active_levels;
    
    // Create SPI communication interface
    Esp32SPI spi(tmc51x0_test_config::SPI_HOST, pin_config, 1000000, active_levels);
    if (!spi.Initialize()) {
        ESP_LOGE(TAG, "Failed to initialize SPI interface");
        return;
    }

    // Create TMC51x0 driver instance
    tmc51x0::TMC51x0<Esp32SPI> driver(spi);
    g_driver = &driver;

    // Configure driver from test rig
    tmc51x0::DriverConfig cfg{};
    tmc51x0_test_config::ConfigureDriverFromTestRig<SELECTED_TEST_RIG>(cfg);
    
    constexpr uint16_t output_full_steps = 
        tmc51x0_test_config::GetTestRigMotorOutputFullSteps<SELECTED_TEST_RIG>();
    
    if (!driver.Initialize(cfg)) {
        ESP_LOGE(TAG, "Failed to initialize TMC51x0 driver");
        return;
    }

    ESP_LOGI(TAG, "Driver initialized successfully");

    // Configure encoder
    tmc51x0::EncoderConfig enc_cfg = 
        tmc51x0_test_config::GetTestRigEncoderConfig<SELECTED_TEST_RIG>();
    
    if (!driver.encoder.Configure(enc_cfg)) {
        ESP_LOGE(TAG, "Failed to configure encoder");
        return;
    }

    driver.encoder.SetResolution(
        tmc51x0_test_config::GetTestRigEncoderPulsesPerRev<SELECTED_TEST_RIG>(),
        tmc51x0_test_config::GetTestRigEncoderInvertDirection<SELECTED_TEST_RIG>());

    // Enable motor
    if (!driver.motorControl.Enable()) {
        ESP_LOGE(TAG, "Failed to enable motor");
        return;
    }

    ESP_LOGI(TAG, "Motor enabled");

    // Calculate steps per revolution (with microsteps)
    float steps_per_rev_with_microsteps = static_cast<float>(output_full_steps) * 256.0f;
    uint16_t full_steps_per_rev = output_full_steps; // Full steps without microsteps

    // Create motion controller (full implementation)
    FatigueTest::FatigueTestMotion motion(&driver);
    motion.ConfigureMotor(static_cast<uint16_t>(steps_per_rev_with_microsteps));
    g_motion = &motion;

    // Wait for config from UI board before finding bounds
    ESP_LOGI(TAG, "Waiting for configuration from UI board...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Give UI board time to send config

    // Find bounds using abstracted bounds finder
    ESP_LOGI(TAG, "Finding bounds using %s method...", g_use_stallguard ? "StallGuard2" : "Encoder");
    
    std::unique_ptr<FatigueTest::IBoundsFinder> bounds_finder;
    if (g_use_stallguard) {
        bounds_finder = FatigueTest::CreateStallGuardBoundsFinder();
    } else {
        bounds_finder = FatigueTest::CreateEncoderBoundsFinder();
    }

    if (bounds_finder) {
        // FindBounds expects full steps per rev (used only for logging, driver handles conversions)
        auto result = bounds_finder->FindBounds(driver, full_steps_per_rev);
        if (result.success) {
            motion.SetGlobalBounds(result.min_bound, result.max_bound);  // Already in degrees
            motion.SetLocalBoundsFromCenterDegrees(-60.0f, 60.0f);
            g_bounds_found = true;
            ESP_LOGI(TAG, "Bounds found using %s: min=%.2f°, max=%.2f° (bounded=%d)", 
                     bounds_finder->GetMethodName(), result.min_bound, result.max_bound, 
                     result.bounded ? 1 : 0);
        } else {
            ESP_LOGW(TAG, "Bounds finding failed, using default bounds");
            motion.SetGlobalBounds(-175.0f, 175.0f);  // Default bounds in degrees
            motion.SetLocalBoundsFromCenterDegrees(-60.0f, 60.0f);
            g_bounds_found = false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to create bounds finder");
        motion.SetGlobalBounds(-175.0f, 175.0f);  // Default bounds in degrees
        motion.SetLocalBoundsFromCenterDegrees(-60.0f, 60.0f);
        g_bounds_found = false;
    }

    // Initialize UART command parser
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              Initializing UART Command Interface                             ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    
    // Use static storage duration to ensure parser lives for the entire program lifetime
    // The parser is used by uart_command_task which runs indefinitely
    static UartCommandParser parser(UART_NUM_0);
    
    // Commands are now automatically routed based on word-based command names
    // Supported commands: set, start, stop, reset, status, help
    // Use 'help' command for usage information
    
    ESP_LOGI(TAG, "UART command interface ready on UART_NUM_0 (USB serial)");
    ESP_LOGI(TAG, "Type 'help' for command usage information");

    // Create tasks
    xTaskCreate(espnow_command_task, "espnow_cmd", 4096, nullptr, 5, nullptr);
    xTaskCreate(motion_control_task, "motion_ctrl", 8192, nullptr, 5, nullptr);
    xTaskCreate(status_update_task, "status_upd", 4096, nullptr, 3, nullptr);
    xTaskCreate(uart_command_task, "uart_cmd", 4096, &parser, 3, nullptr);
    
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                    System Ready - Use UART or ESP-NOW Commands                ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    // Main loop (also tracks time elapsed)
    const char* task_name = "main_loop";
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_log_time_us = start_time_us;
    const int64_t log_interval_us = 5000000; // Log every 5 seconds
    
    ESP_LOGI(TAG, "[%s] Main loop started", task_name);
    
    while (true) {
        int64_t current_time_us = esp_timer_get_time();
        
        // Log elapsed time periodically
        if (current_time_us - last_log_time_us >= log_interval_us) {
            int64_t elapsed_ms = (current_time_us - start_time_us) / 1000;
            ESP_LOGI(TAG, "[%s] Time elapsed: %lld.%03lld seconds (active)", 
                     task_name, elapsed_ms / 1000, elapsed_ms % 1000);
            last_log_time_us = current_time_us;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
