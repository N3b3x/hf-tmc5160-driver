/**
 * @file ui_oled.cpp
 * @brief OLED-based UI implementation with rotary encoder navigation
 */

#include "ui_oled.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "settings.hpp"
#include "espnow_protocol.hpp"
#include <functional>
#include "../../components/Adafruit_BusIO_ESPIDF/Wire.h"
#include "../../components/Adafruit_BusIO_ESPIDF/Arduino.h"

static const char* TAG_UI_OLED = "UI_OLED";

// Display and input devices
static Adafruit_SH1106* g_display = nullptr;
static EC11Encoder* g_encoder = nullptr;
static OLEDMenu* g_menu = nullptr;

// Shared pointers
static QueueHandle_t s_uiQueue = nullptr;
static Settings* s_settings = nullptr;
static uint32_t* s_lastActivityTick = nullptr;

// Popup state
static char s_popupMsg[32] = {0};
static std::function<void(bool)> s_popupCallback = nullptr;
static bool s_popupYesSelected = false;

// UI state
static UiState s_state = UiState::MAIN;
static uint32_t s_currentCycle = 0;
static uint8_t s_errorCode = 0;
static uint32_t s_lastStatusTick = 0;
static bool s_settingsSynced = false;  // Track if settings have been synced with test unit

// Mark user activity
static void touch_activity() {
    if (s_lastActivityTick) {
        *s_lastActivityTick = xTaskGetTickCount();
    }
}

// Forward declarations
static void handle_button(const ButtonEvent& be);
static void handle_proto(const ProtoEvent& pe);
static void draw_popup();
static void draw_main_screen();
static void draw_running_screen();
static void draw_paused_screen();
static void draw_complete_screen();
static void draw_error_screen();

void UI_OLED::init(QueueHandle_t ui_queue, Settings* settings, uint32_t* inactivity_ticks_ptr) {
    s_uiQueue = ui_queue;
    s_settings = settings;
    s_lastActivityTick = inactivity_ticks_ptr;

    ESP_LOGI(TAG_UI_OLED, "Initializing OLED UI system...");

    // Configure I2C pins for OLED
    Adafruit_I2CDevice::setDefaultPins(OLED_SDA_PIN, OLED_SCL_PIN);
    Adafruit_I2CDevice::setDefaultFrequency(OLED_I2C_FREQ);

    // I2C is initialized automatically by Adafruit_I2CDevice when first device begins
    // No need to call Wire.begin() explicitly

    // Initialize OLED display
    g_display = new Adafruit_SH1106(OLED_WIDTH, OLED_HEIGHT, &Wire, -1, OLED_I2C_ADDR);
    // Small delay before first init to avoid early I2C timeouts on some modules
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!g_display->begin(OLED_I2C_ADDR, true)) {
        ESP_LOGW(TAG_UI_OLED, "OLED init failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
        if (!g_display->begin(OLED_I2C_ADDR, true)) {
            ESP_LOGE(TAG_UI_OLED, "Failed to initialize OLED display after retry");
            return;
        }
    }
    
    // Apply saved rotation setting
    if (s_settings) {
        g_display->setRotation(s_settings->ui.orientation_flipped ? 2 : 0);
    }
    
    ESP_LOGI(TAG_UI_OLED, "OLED display initialized: %dx%d, I2C addr=0x%02X", 
             OLED_WIDTH, OLED_HEIGHT, OLED_I2C_ADDR);

    // Initialize EC11 encoder
    g_encoder = new EC11Encoder(ENCODER_TRA_PIN, ENCODER_TRB_PIN, ENCODER_PSH_PIN, 
                                ENCODER_PULSES_PER_REV);
    if (!g_encoder->begin()) {
        ESP_LOGE(TAG_UI_OLED, "Failed to initialize EC11 encoder");
        return;
    }
    ESP_LOGI(TAG_UI_OLED, "EC11 encoder initialized");

    // Initialize menu system
    g_menu = new OLEDMenu(g_display, g_encoder, s_settings);
    if (!g_menu->begin()) {
        ESP_LOGE(TAG_UI_OLED, "Failed to initialize menu system");
        return;
    }
    ESP_LOGI(TAG_UI_OLED, "Menu system initialized");

    // Draw initial screen
    draw_main_screen();
    touch_activity();
}

void UI_OLED::task(void* arg) {
    TickType_t lastMenuUpdate = 0;
    const TickType_t menuUpdateInterval = pdMS_TO_TICKS(50); // Update menu every 50ms
    
    // Track encoder position for popup navigation
    static int32_t last_popup_pos = 0;
    // Track encoder button for non-menu states
    static bool last_enc_btn = false;

    while (true) {
        // Update menu system (processes encoder events)
        TickType_t now = xTaskGetTickCount();
        if (now - lastMenuUpdate >= menuUpdateInterval) {
            if (g_menu && s_state == UiState::SETTINGS_MENU) {
                if (g_menu->update()) {
                    touch_activity();
                }
            } else if (g_encoder) {
                // Poll encoder button for non-menu states (menu handles it internally)
                bool curr_enc_btn = g_encoder->isButtonPressed();
                if (curr_enc_btn && !last_enc_btn) {
                    touch_activity();
                    if (s_state == UiState::MAIN) {
                        // Encoder button enters settings menu
                        s_state = UiState::SETTINGS_MENU;
                        if (g_menu) {
                            g_menu->resetInputState();
                            g_menu->refresh();
                        }
                    } else {
                        // In other states, treat encoder button like CONFIRM
                        ButtonEvent be{ ButtonId::CONFIRM };
                        handle_button(be);
                    }
                }
                last_enc_btn = curr_enc_btn;

                if (s_state == UiState::CONFIRM_POPUP) {
                    // Handle popup navigation
                    int32_t current_pos = g_encoder->getPosition() / 4;
                    if (current_pos != last_popup_pos) {
                        s_popupYesSelected = !s_popupYesSelected;
                        draw_popup();
                        touch_activity();
                        last_popup_pos = current_pos;
                    }
                } else {
                    // Keep tracking position to avoid jumps when entering other states
                    last_popup_pos = g_encoder->getPosition() / 4;
                }
            }
            lastMenuUpdate = now;
        }

        // Process UI events
        UiEvent ev{};
        // Use non-blocking receive to prevent freezing the UI loop
        if (xQueueReceive(s_uiQueue, &ev, 0) == pdTRUE) {
            switch (ev.type) {
                case UiEventType::BTN:
                    touch_activity();
                    handle_button(ev.data.btn);
                    break;
                case UiEventType::PROTO:
                    touch_activity();
                    handle_proto(ev.data.proto);
                    break;
                case UiEventType::TIMER_INACTIVITY:
                    // Handled by main power task
                    break;
            }
        }

        // Update running/paused screens periodically
        if (s_state == UiState::RUNNING || s_state == UiState::PAUSED) {
            static TickType_t lastStatusUpdate = 0;
            if (xTaskGetTickCount() - lastStatusUpdate >= pdMS_TO_TICKS(1000)) {
                if (s_state == UiState::RUNNING) {
                    draw_running_screen();
                } else if (s_state == UiState::PAUSED) {
                    draw_paused_screen();
                }
                lastStatusUpdate = xTaskGetTickCount();
            }
        }
        
        // Control loop rate (~100Hz) to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void show_popup(const char* msg, std::function<void(bool)> cb) {
    strncpy(s_popupMsg, msg, sizeof(s_popupMsg) - 1);
    s_popupCallback = cb;
    s_popupYesSelected = false; // Default to NO
    s_state = UiState::CONFIRM_POPUP;
    draw_popup();
}

static void handle_button(const ButtonEvent& be) {
    ESP_LOGI(TAG_UI_OLED, "UI Button event: ID=%d", (int)be.id);
    
    // Handle Popup
    if (s_state == UiState::CONFIRM_POPUP) {
        bool confirmed = false;
        if (be.id == ButtonId::CONFIRM) {
            confirmed = s_popupYesSelected;
        } else if (be.id == ButtonId::BACK) {
            confirmed = false;
        } else {
            return; // Ignore other buttons
        }
        
        if (s_popupCallback) {
            s_popupCallback(confirmed);
        }
        // If callback didn't change state, go back to MAIN as safe default
        if (s_state == UiState::CONFIRM_POPUP) {
            s_state = UiState::MAIN;
            draw_main_screen();
        }
        return;
    }

    if (!g_menu) return;

    // Handle menu navigation buttons
    if (s_state == UiState::SETTINGS_MENU) {
        if (!g_menu->handleButton(be.id)) {
            // Menu exited (returned false) - save and send settings
            if (s_settings) {
                // Save to flash (includes both test machine and UI settings)
                SettingsStore::save(*s_settings);
                // Send to test unit (test machine settings only - orientation_flipped is UI-only)
                // Note: Protocol still sends orientation_flipped, but test unit should ignore it
                s_settingsSynced = false;
                EspNowProto::send_config_set(*s_settings);
                ESP_LOGI(TAG_UI_OLED, "Settings saved and sent to test unit, waiting for acknowledgment");
            }
            s_state = UiState::MAIN;
            draw_main_screen();
        }
        return;
    }

    // Handle other states
    switch (be.id) {
        case ButtonId::BACK:
            if (s_state == UiState::SETTINGS_MENU) {
                s_state = UiState::MAIN;
                draw_main_screen();
            } else if (s_state == UiState::RUNNING || s_state == UiState::PAUSED) {
                // Stop confirmation
                show_popup("Stop Test?", [](bool yes){
                    if (yes) {
                        EspNowProto::send_stop();
                        s_state = UiState::MAIN;
                        draw_main_screen();
                    } else {
                        // Return to previous state
                        s_state = UiState::RUNNING; // Assume running for now
                        draw_running_screen();
                    }
                });
            }
            break;
        case ButtonId::CONFIRM:
            if (s_state == UiState::MAIN) {
                // Start Test Confirmation
                show_popup("Start Test?", [](bool yes){
                    if (yes) {
                        EspNowProto::send_start();
                        s_state = UiState::RUNNING;
                        draw_running_screen();
                    } else {
                        s_state = UiState::MAIN;
                        draw_main_screen();
                    }
                });
            } else if (s_state == UiState::RUNNING) {
                // Pause
                EspNowProto::send_pause();
                s_state = UiState::PAUSED;
                draw_paused_screen();
            } else if (s_state == UiState::PAUSED) {
                // Resume
                EspNowProto::send_resume();
                s_state = UiState::RUNNING;
                draw_running_screen();
            }
            break;
        default:
            break;
    }
}

static void handle_proto(const ProtoEvent& pe) {
    // Update connection timestamp for any protocol event (not just STATUS)
    // This ensures we detect connection from any communication with test unit
    s_lastStatusTick = xTaskGetTickCount();
    
    switch (pe.type) {
        case ProtoEventType::CONFIG_UPDATED:
            // Settings received from test unit - update only TEST MACHINE settings
            // UI-only settings are preserved and NOT overwritten
            s_settingsSynced = true;
            if (s_settings) {
                TestUnitSettings test_unit_settings = pe.data.config;
                
                // Update only test machine settings from test unit
                bool test_settings_changed = false;
                if (s_settings->test_unit.cycle_amount != test_unit_settings.cycle_amount ||
                    s_settings->test_unit.time_per_cycle != test_unit_settings.time_per_cycle ||
                    s_settings->test_unit.dwell_time != test_unit_settings.dwell_time ||
                    s_settings->test_unit.bounds_method_stallguard != test_unit_settings.bounds_method_stallguard) {
                    test_settings_changed = true;
                    // Update test machine settings only (UI settings preserved automatically)
                    s_settings->test_unit = test_unit_settings;
                    // Save to NVS to keep them in sync
                    SettingsStore::save(*s_settings);
                    ESP_LOGI(TAG_UI_OLED, "Test machine settings updated from test unit");
                } else {
                    ESP_LOGI(TAG_UI_OLED, "Test machine settings match, no update needed");
                }
                // Always update display rotation from our preserved UI setting
                if (g_display) {
                    g_display->setRotation(s_settings->ui.orientation_flipped ? 2 : 0);
                }
            }
            ESP_LOGI(TAG_UI_OLED, "Settings synced with test unit (UI settings preserved)");
            draw_main_screen();
            break;
        case ProtoEventType::CONFIG_APPLY_OK:
            // Settings successfully applied - mark as synced
            s_settingsSynced = true;
            ESP_LOGI(TAG_UI_OLED, "Settings applied successfully, synced=true");
            draw_main_screen();
            break;
        case ProtoEventType::CONFIG_APPLY_FAIL:
            s_errorCode = 1;
            s_state = UiState::ERROR_SCREEN;
            ESP_LOGE(TAG_UI_OLED, "Settings apply failed");
            draw_error_screen();
            break;
        case ProtoEventType::STATUS:
            if (pe.data.status.cycle != s_currentCycle) {
                s_currentCycle = pe.data.status.cycle;
                if (s_state == UiState::RUNNING) {
                    draw_running_screen();
                }
            }
            break;
        case ProtoEventType::STARTED:
            s_state = UiState::RUNNING;
            draw_running_screen();
            break;
        case ProtoEventType::PAUSED:
            s_state = UiState::PAUSED;
            draw_paused_screen();
            break;
        case ProtoEventType::RESUMED:
            s_state = UiState::RUNNING;
            draw_running_screen();
            break;
        case ProtoEventType::STOPPED:
            s_state = UiState::MAIN;
            draw_main_screen();
            break;
        case ProtoEventType::TEST_COMPLETED:
            s_state = UiState::COMPLETE;
            draw_complete_screen();
            break;
        case ProtoEventType::ERROR_EVENT:
            s_state = UiState::ERROR_SCREEN;
            s_errorCode = pe.data.error.err_code;
            draw_error_screen();
            break;
        default:
            break;
    }
}

static void draw_popup() {
    if (!g_display) return;

    g_display->clearDisplay();
    
    // Draw border
    g_display->drawRect(0, 0, 128, 64, 1);
    g_display->drawRect(2, 2, 124, 60, 1);
    
    // Title
    g_display->setTextSize(1);
    g_display->setTextColor(1);
    g_display->setCursor(10, 8);
    g_display->print("CONFIRMATION");
    g_display->drawLine(10, 18, 118, 18, 1);
    
    // Message
    // Center the message roughly
    int len = strlen(s_popupMsg);
    int x = (128 - (len * 6)) / 2;
    if (x < 4) x = 4;
    g_display->setCursor(x, 25);
    g_display->print(s_popupMsg);
    
    // Buttons
    // NO (Back)
    if (!s_popupYesSelected) {
        g_display->fillRect(8, 43, 55, 12, 1);
        g_display->setTextColor(0);
    } else {
        g_display->setTextColor(1);
    }
    g_display->setCursor(10, 45);
    g_display->print("NO (Back)");
    
    // YES (Ok)
    if (s_popupYesSelected) {
        g_display->fillRect(68, 43, 55, 12, 1);
        g_display->setTextColor(0);
    } else {
        g_display->setTextColor(1);
    }
    g_display->setCursor(70, 45);
    g_display->print("YES (Ok)");
    
    g_display->display();
}

static void draw_main_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    
    // Header (white background)
    g_display->fillRect(0, 0, 128, 12, 1);
    g_display->setTextColor(0);
    g_display->setTextSize(1);
    g_display->setCursor(2, 2);
    g_display->print("Fatigue Test");  // Shortened
    
    // Connection Status indicator (right side of header, black on white)
    TickType_t now_ticks = xTaskGetTickCount();
    // Check if we've received any communication (s_lastStatusTick was set)
    // If 0, we haven't received anything yet, so show as disconnected
    bool connected = (s_lastStatusTick > 0) && (now_ticks - s_lastStatusTick < pdMS_TO_TICKS(5000));
    if (connected) {
        // Connected: solid black dot on white header
        g_display->fillCircle(120, 6, 3, 0);
    } else {
        // Disconnected: hollow black circle with X
        g_display->drawCircle(120, 6, 3, 0);
        g_display->drawLine(118, 4, 122, 8, 0);
        g_display->drawLine(122, 4, 118, 8, 0);
    }
    
    // Status Section
    g_display->setTextColor(1);
    g_display->setCursor(0, 16);
    g_display->print("Status: ");
    // Show status based on test state, connection, and settings sync
    if (s_state == UiState::RUNNING) {
        g_display->print("RUNNING");
    } else if (s_state == UiState::PAUSED) {
        g_display->print("PAUSED");
    } else {
        // In MAIN state - show connection/sync status
        TickType_t now_ticks = xTaskGetTickCount();
        // Check if we've received any communication (s_lastStatusTick was set)
        // If 0, we haven't received anything yet, so show as disconnected
        bool connected = (s_lastStatusTick > 0) && (now_ticks - s_lastStatusTick < pdMS_TO_TICKS(5000));
        if (connected && s_settingsSynced) {
            g_display->print("READY");
        } else if (connected) {
            g_display->print("SYNCING");
        } else {
            g_display->print("DISCONNECTED");
        }
    }
    
    // Data Section
    g_display->drawLine(0, 26, 128, 26, 1);
    
    g_display->setCursor(0, 30);
    g_display->print("Cycles:");
    g_display->setCursor(64, 30);
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", s_currentCycle);
    g_display->print(buf);
    
    g_display->setCursor(0, 42);
    g_display->print("Target:");
    g_display->setCursor(64, 42);
    if (s_settings) {
        snprintf(buf, sizeof(buf), "%lu", s_settings->test_unit.cycle_amount);
        g_display->print(buf);
    } else {
        g_display->print("---");
    }

    // Footer / Instructions
    g_display->drawLine(0, 52, 128, 52, 1);
    g_display->setCursor(0, 55);
    g_display->print("START:OK MENU:Enc");
    
    g_display->display();
}

static void draw_running_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    
    // Header
    g_display->fillRect(0, 0, 128, 12, 1);
    g_display->setTextColor(0);
    g_display->setTextSize(1);
    g_display->setCursor(40, 2);
    g_display->print("RUNNING");
    
    // Data
    g_display->setTextColor(1);
    g_display->setTextSize(2);
    
    // Center the cycle count
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", s_currentCycle);
    int16_t x1, y1;
    uint16_t w, h;
    g_display->getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor((128 - w) / 2, 25);
    g_display->print(buf);
    
    g_display->setTextSize(1);
    g_display->setCursor(20, 45);
    g_display->print("Target: ");
    if (s_settings) {
        snprintf(buf, sizeof(buf), "%lu", s_settings->test_unit.cycle_amount);
        g_display->print(buf);
    }
    
    // Footer
    g_display->drawLine(0, 54, 128, 54, 1);
    g_display->setCursor(0, 56);
    g_display->print("OK:Pause  BACK:Stop");
    
    g_display->display();
}

static void draw_paused_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    
    // Header
    g_display->drawRect(0, 0, 128, 12, 1);
    g_display->setTextColor(1);
    g_display->setTextSize(1);
    g_display->setCursor(45, 2);
    g_display->print("PAUSED");
    
    // Data
    g_display->setTextSize(2);
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", s_currentCycle);
    int16_t x1, y1;
    uint16_t w, h;
    g_display->getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor((128 - w) / 2, 25);
    g_display->print(buf);
    
    g_display->setTextSize(1);
    g_display->setCursor(20, 45);
    g_display->print("Target: ");
    if (s_settings) {
        snprintf(buf, sizeof(buf), "%lu", s_settings->test_unit.cycle_amount);
        g_display->print(buf);
    }
    
    // Footer
    g_display->drawLine(0, 54, 128, 54, 1);
    g_display->setCursor(0, 56);
    g_display->print("OK:Resume BACK:Stop");
    
    g_display->display();
}

static void draw_complete_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    g_display->setTextSize(1);
    g_display->setTextColor(1);
    
    // Title
    g_display->setCursor(20, 0);
    g_display->print("Test Complete");
    
    // Final cycle count
    g_display->setCursor(0, 20);
    g_display->print("Total Cycles: ");
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", s_currentCycle);
    g_display->print(buf);
    
    g_display->display();
}

static void draw_error_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    g_display->setTextSize(1);
    g_display->setTextColor(1);
    
    // Title
    g_display->setCursor(30, 0);
    g_display->print("ERROR");
    
    // Error code
    g_display->setCursor(0, 20);
    g_display->print("Code: ");
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", s_errorCode);
    g_display->print(buf);
    
    g_display->display();
}

void UI_OLED::prepareForSleep() {
    if (g_display) {
        g_display->clearDisplay();
        g_display->setTextSize(1);
        g_display->setTextColor(1);
        g_display->setCursor(20, 25);
        g_display->print("Sleeping...");
        g_display->display();
        // Give it a moment to render before sleep cuts power/clocks
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Configure wake-up sources for ESP32-C6
    // NOTE: GPIO 22 is NOT a valid deep sleep wakeup pin on ESP32-C6
    // Only use the physical BACK and CONFIRM buttons for wakeup
    
    ESP_LOGI(TAG_UI_OLED, "Configuring GPIO wakeup sources...");
    
    // Configure only valid wakeup GPIOs (exclude encoder push - GPIO 22)
    gpio_num_t wakeup_pins[] = {BTN_BACK_GPIO, BTN_CONFIRM_GPIO};
    
    for (int i = 0; i < 2; i++) {
        gpio_num_t pin = wakeup_pins[i];
        
        // Configure as input with pull-up (buttons are active-low)
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        
        ESP_LOGI(TAG_UI_OLED, "Configured GPIO %d for wakeup (current level: %d)", 
                 pin, gpio_get_level(pin));
    }
    
    // Build wakeup mask (only BACK and CONFIRM buttons)
    uint64_t wakeup_mask = (1ULL << BTN_BACK_GPIO) | (1ULL << BTN_CONFIRM_GPIO);
    
    // Enable deep sleep wakeup on LOW level (button press)
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(wakeup_mask, ESP_GPIO_WAKEUP_GPIO_LOW));
    
    // CRITICAL: Enable the GPIO wakeup source globally
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    
    ESP_LOGI(TAG_UI_OLED, "Sleep prepared. Wakeup mask: 0x%llx", wakeup_mask);
    ESP_LOGI(TAG_UI_OLED, "GPIO 6 (Back): %d, GPIO 4 (Confirm): %d",
             gpio_get_level(BTN_BACK_GPIO), 
             gpio_get_level(BTN_CONFIRM_GPIO));
}

