/**
 * @file ui_oled.cpp
 * @brief OLED-based UI implementation with rotary encoder navigation
 */

#include "ui_oled.hpp"
#include "esp_log.h"
#include "esp_timer.h"
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

// UI state
static UiState s_state = UiState::MAIN;
static uint32_t s_currentCycle = 0;
static uint8_t s_errorCode = 0;

// Mark user activity
static void touch_activity() {
    if (s_lastActivityTick) {
        *s_lastActivityTick = xTaskGetTickCount();
    }
}

// Forward declarations
static void handle_button(const ButtonEvent& be);
static void handle_proto(const ProtoEvent& pe);
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
    if (!g_display->begin(OLED_I2C_ADDR, true)) {
        ESP_LOGE(TAG_UI_OLED, "Failed to initialize OLED display");
        return;
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

    while (true) {
        // Update menu system (processes encoder events)
        TickType_t now = xTaskGetTickCount();
        if (now - lastMenuUpdate >= menuUpdateInterval) {
            if (g_menu && s_state == UiState::SETTINGS_MENU) {
                if (g_menu->update()) {
                    touch_activity();
                }
            }
            lastMenuUpdate = now;
        }

        // Process UI events
        UiEvent ev{};
        if (xQueueReceive(s_uiQueue, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
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
    }
}

static void handle_button(const ButtonEvent& be) {
    ESP_LOGI(TAG_UI_OLED, "UI Button event: ID=%d", (int)be.id);
    if (!g_menu) return;

    // Handle menu navigation buttons
    if (s_state == UiState::SETTINGS_MENU) {
        g_menu->handleButton(be.id);
        return;
    }

    // Handle other states
    switch (be.id) {
        case ButtonId::BACK:
            if (s_state == UiState::SETTINGS_MENU) {
                s_state = UiState::MAIN;
                draw_main_screen();
            }
            break;
        case ButtonId::CONFIRM:
            if (s_state == UiState::MAIN) {
                s_state = UiState::SETTINGS_MENU;
                if (g_menu) {
                    g_menu->refresh();
                }
            }
            break;
        default:
            break;
    }
}

static void handle_proto(const ProtoEvent& pe) {
    switch (pe.type) {
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

static void draw_main_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    g_display->setTextSize(1);
    g_display->setTextColor(1);
    
    // Title
    g_display->setCursor(20, 0);
    g_display->print("Fatigue Tester");
    
    // Status
    g_display->setCursor(0, 15);
    g_display->print("Status: Ready");
    
    // Instructions
    g_display->setCursor(0, 30);
    g_display->print("Rotate: Navigate");
    g_display->setCursor(0, 42);
    g_display->print("Press: Select");
    g_display->setCursor(0, 54);
    g_display->print("CONFIRM: Settings");
    
    g_display->display();
}

static void draw_running_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    g_display->setTextSize(1);
    g_display->setTextColor(1);
    
    // Title
    g_display->setCursor(30, 0);
    g_display->print("Running");
    
    // Cycle count
    g_display->setCursor(0, 20);
    g_display->print("Cycles: ");
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", s_currentCycle);
    g_display->print(buf);
    
    // Status
    g_display->setCursor(0, 35);
    g_display->print("Status: Active");
    
    g_display->display();
}

static void draw_paused_screen() {
    if (!g_display) return;

    g_display->clearDisplay();
    g_display->setTextSize(1);
    g_display->setTextColor(1);
    
    // Title
    g_display->setCursor(30, 0);
    g_display->print("Paused");
    
    // Cycle count
    g_display->setCursor(0, 20);
    g_display->print("Cycles: ");
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", s_currentCycle);
    g_display->print(buf);
    
    // Status
    g_display->setCursor(0, 35);
    g_display->print("Status: Paused");
    
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

