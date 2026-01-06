/**
 * @file espnow_protocol_test_unit.cpp
 * @brief Minimal ESP-NOW protocol test unit - no motor control
 * 
 * This is a simplified test unit that:
 * - Receives ESP-NOW commands from remote controller
 * - Responds with appropriate protocol messages
 * - Does NOT initialize motor driver, bounds finder, or motion controller
 * - Useful for testing ESP-NOW protocol communication without hardware dependencies
 */

#include "espnow_protocol.hpp"
#include "espnow_receiver.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char* TAG = "EspNowTestUnit";

// Global state
static Settings g_settings{};
static QueueHandle_t g_espnowQueue = nullptr;
static TestState g_current_state = TestState::IDLE;
static uint32_t g_simulated_cycle = 0;

/**
 * @brief FreeRTOS task: handle incoming ESP-NOW protocol events (test-unit simulator).
 *
 * @details
 * This task consumes `ProtoEvent` messages from `g_espnowQueue` and responds with
 * protocol ACKs and status updates. It does not initialize motor hardware.
 *
 * @param arg Unused (FreeRTOS task signature).
 */
static void espnow_command_task(void* arg)
{
    ESP_LOGI(TAG, "ESP-NOW command task started");
    
    ProtoEvent ev{};
    while (true) {
        if (xQueueReceive(g_espnowQueue, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (ev.type) {
                case ProtoEventType::CONFIG_REQUEST:
                    ESP_LOGI(TAG, "✓ CONFIG_REQUEST received");
                    EspNowReceiver::send_config_response(g_settings);
                    ESP_LOGI(TAG, "  → Sent CONFIG_RESPONSE");
                    break;
                    
                case ProtoEventType::CONFIG_SET:
                    ESP_LOGI(TAG, "✓ CONFIG_SET received:");
                    ESP_LOGI(TAG, "    cycles=%u, time_per_cycle=%u sec, dwell=%u sec, bounds_method=%s",
                             ev.data.config.cycle_amount,
                             ev.data.config.time_per_cycle,
                             ev.data.config.dwell_time,
                             ev.data.config.bounds_method_stallguard ? "StallGuard" : "Encoder");
                    
                    // Store settings (but don't actually configure anything)
                    g_settings.test_unit.cycle_amount = ev.data.config.cycle_amount;
                    g_settings.test_unit.time_per_cycle = ev.data.config.time_per_cycle;
                    g_settings.test_unit.dwell_time = ev.data.config.dwell_time;
                    g_settings.test_unit.bounds_method_stallguard = ev.data.config.bounds_method_stallguard;
                    
                    EspNowReceiver::send_config_ack(true, 0);
                    ESP_LOGI(TAG, "  → Sent CONFIG_ACK (success)");
                    break;
                    
                case ProtoEventType::START:
                    ESP_LOGI(TAG, "✓ START command received");
                    if (g_current_state == TestState::RUNNING) {
                        ESP_LOGW(TAG, "  Already running, ignoring");
                        EspNowReceiver::send_start_ack();
                        break;
                    }
                    
                    // Simulate starting (no actual motion)
                    g_current_state = TestState::RUNNING;
                    g_simulated_cycle = 0;
                    EspNowReceiver::send_start_ack();
                    EspNowReceiver::send_status_update(0, TestState::RUNNING);
                    ESP_LOGI(TAG, "  → Sent START_ACK and STATUS_UPDATE (RUNNING)");
                    break;
                    
                case ProtoEventType::PAUSE:
                    ESP_LOGI(TAG, "✓ PAUSE command received");
                    if (g_current_state == TestState::RUNNING) {
                        g_current_state = TestState::PAUSED;
                        EspNowReceiver::send_pause_ack();
                        EspNowReceiver::send_status_update(g_simulated_cycle, TestState::PAUSED);
                        ESP_LOGI(TAG, "  → Sent PAUSE_ACK and STATUS_UPDATE (PAUSED)");
                    } else {
                        ESP_LOGW(TAG, "  Not running, ignoring");
                        EspNowReceiver::send_pause_ack();
                    }
                    break;
                    
                case ProtoEventType::RESUME:
                    ESP_LOGI(TAG, "✓ RESUME command received");
                    if (g_current_state == TestState::PAUSED) {
                        g_current_state = TestState::RUNNING;
                        EspNowReceiver::send_resume_ack();
                        EspNowReceiver::send_status_update(g_simulated_cycle, TestState::RUNNING);
                        ESP_LOGI(TAG, "  → Sent RESUME_ACK and STATUS_UPDATE (RUNNING)");
                    } else {
                        ESP_LOGW(TAG, "  Not paused, ignoring");
                        EspNowReceiver::send_resume_ack();
                    }
                    break;
                    
                case ProtoEventType::STOP:
                    ESP_LOGI(TAG, "✓ STOP command received");
                    g_current_state = TestState::IDLE;
                    EspNowReceiver::send_stop_ack();
                    EspNowReceiver::send_status_update(g_simulated_cycle, TestState::IDLE);
                    ESP_LOGI(TAG, "  → Sent STOP_ACK and STATUS_UPDATE (IDLE)");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unhandled event type: %d", (int)ev.type);
                    break;
            }
        }
    }
}

/**
 * @brief FreeRTOS task: periodically send simulated STATUS_UPDATE messages.
 *
 * @details
 * When the simulated state is RUNNING, this task increments cycle count on a
 * simple timer and sends STATUS_UPDATE frames. When the configured target is
 * reached, it sends COMPLETED and TEST_COMPLETE.
 *
 * @param arg Unused (FreeRTOS task signature).
 */
static void status_update_task(void* arg)
{
    ESP_LOGI(TAG, "Status update task started");
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
        
        if (g_current_state == TestState::RUNNING) {
            // Simulate cycle increment (increment every 2 seconds for demo)
            static uint32_t last_cycle_update = 0;
            uint32_t now = esp_timer_get_time() / 1000000; // seconds
            if (now - last_cycle_update >= 2) {
                g_simulated_cycle++;
                last_cycle_update = now;
                
                // Check if test should complete
                if (g_settings.test_unit.cycle_amount > 0 && 
                    g_simulated_cycle >= g_settings.test_unit.cycle_amount) {
                    g_current_state = TestState::COMPLETED;
                    EspNowReceiver::send_status_update(g_simulated_cycle, TestState::COMPLETED);
                    EspNowReceiver::send_test_complete();
                    ESP_LOGI(TAG, "✓ Test completed (simulated): %u cycles", g_simulated_cycle);
                } else {
                    EspNowReceiver::send_status_update(g_simulated_cycle, TestState::RUNNING);
                }
            } else {
                // Just send status update without incrementing
                EspNowReceiver::send_status_update(g_simulated_cycle, TestState::RUNNING);
            }
        }
    }
}

extern "C" void app_main()
{
    /**
     * @brief ESP-IDF application entry point for the protocol test unit.
     *
     * @details
     * Initializes ESP-NOW receiver and starts background tasks that simulate the
     * fatigue test unit protocol behavior without any motor driver dependencies.
     */
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         ESP-NOW Protocol Test Unit (No Motor Control)                        ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "This unit will:");
    ESP_LOGI(TAG, "  • Receive ESP-NOW commands from remote controller");
    ESP_LOGI(TAG, "  • Respond with appropriate protocol messages");
    ESP_LOGI(TAG, "  • Simulate test state changes (no actual motor control)");
    ESP_LOGI(TAG, "  • Send periodic status updates when 'running'");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Use this to test ESP-NOW protocol communication without hardware dependencies.");
    ESP_LOGI(TAG, "");

    // Initialize ESP-NOW receiver
    g_espnowQueue = xQueueCreate(10, sizeof(ProtoEvent));
    if (!EspNowReceiver::init(g_espnowQueue)) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW receiver");
        return;
    }

    // Initialize default settings
    g_settings.test_unit.cycle_amount = 1000;
    g_settings.test_unit.time_per_cycle = 1;
    g_settings.test_unit.dwell_time = 1;
    g_settings.test_unit.bounds_method_stallguard = true;
    g_current_state = TestState::IDLE;
    g_simulated_cycle = 0;

    ESP_LOGI(TAG, "Default settings:");
    ESP_LOGI(TAG, "  cycles=%u, time_per_cycle=%u sec, dwell=%u sec, bounds_method=%s",
             g_settings.test_unit.cycle_amount,
             g_settings.test_unit.time_per_cycle,
             g_settings.test_unit.dwell_time,
             g_settings.test_unit.bounds_method_stallguard ? "StallGuard" : "Encoder");

    // Create tasks
    ESP_LOGI(TAG, "Creating background tasks...");
    xTaskCreate(espnow_command_task, "espnow_cmd", 4096, nullptr, 5, nullptr);
    xTaskCreate(status_update_task, "status_upd", 4096, nullptr, 3, nullptr);
    ESP_LOGI(TAG, "All tasks created");
    
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                    System Ready - Waiting for ESP-NOW Commands                ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Periodic status log
        static uint32_t last_status_log = 0;
        uint32_t now = esp_timer_get_time() / 1000000;
        if (now - last_status_log >= 10) { // Log every 10 seconds
            const char* state_str = "UNKNOWN";
            switch (g_current_state) {
                case TestState::IDLE: state_str = "IDLE"; break;
                case TestState::RUNNING: state_str = "RUNNING"; break;
                case TestState::PAUSED: state_str = "PAUSED"; break;
                case TestState::COMPLETED: state_str = "COMPLETED"; break;
                case TestState::ERROR: state_str = "ERROR"; break;
            }
            ESP_LOGI(TAG, "Status: state=%s, cycle=%u/%u", 
                     state_str, g_simulated_cycle, g_settings.test_unit.cycle_amount);
            last_status_log = now;
        }
    }
}
