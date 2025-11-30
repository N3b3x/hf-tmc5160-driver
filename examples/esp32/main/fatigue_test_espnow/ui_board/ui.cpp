/**
 * @file ui.cpp
 * @brief UI implementation with Adafruit 2.9" ThinkInk E-Ink display
 */

#include "ui.hpp"
#include "espnow_protocol.hpp"
#include "settings.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "esp_timer.h"

// Adafruit E-Ink display libraries
#include <Adafruit_GFX.h>
#include <Adafruit_EPD.h>

static const char* TAG_UI = "UI";

// E-Ink: 2.9" ThinkInk FeatherWing (296 x 128 pixels)
// Uses IL0373 controller (standard for FeatherWing)
static Adafruit_IL0373* g_display = nullptr;

// Shared pointers
static QueueHandle_t s_uiQueue        = nullptr;
static Settings*     s_settings       = nullptr;
static uint32_t*     s_lastActivityTick = nullptr;

// State
static UiState   s_state       = UiState::MAIN;
static uint32_t  s_currentCycle= 0;
static bool      s_errorBlink  = false;
static uint8_t   s_errorCode   = 0;

// Forward drawing helpers
static void draw_main_screen();
static void draw_settings_screen();
static void draw_confirm_stop();
static void draw_error_screen();
static void draw_complete_screen();
static void update_status_footer();

// Display helper functions
static void set_title_style();
static void set_header_style();
static void set_button_style(bool selected = false);
static void clear_screen();

// Mark user activity (for inactivity timer)
static void touch_activity()
{
    if (s_lastActivityTick) {
        *s_lastActivityTick = xTaskGetTickCount();
    }
}

// -------------- PUBLIC INIT & TASK ---------------

void UI::init(QueueHandle_t ui_queue, Settings* settings, uint32_t* inactivity_ticks_ptr)
{
    s_uiQueue          = ui_queue;
    s_settings         = settings;
    s_lastActivityTick = inactivity_ticks_ptr;

    // Initialize e-ink display
    g_display = new Adafruit_IL0373(
        EINK_CS_PIN,
        EINK_DC_PIN,
        EINK_RESET_PIN,
        EINK_BUSY_PIN,
        296,   // width
        128    // height
    );

    if (!g_display->begin()) {
        ESP_LOGE(TAG_UI, "Failed to initialize e-ink display!");
        // Continue without display - will log errors
        return;
    }

    ESP_LOGI(TAG_UI, "E-ink display initialized");

    // Set rotation based on settings
    g_display->setRotation(s_settings->orientation_flipped ? 2 : 0);

    // Initial screen draw
    draw_main_screen();
    touch_activity();
}

static void handle_button(const ButtonEvent& be);
static void handle_proto(const ProtoEvent& pe);

void UI::task(void* arg)
{
    UiEvent ev{};
    while (true) {
        if (xQueueReceive(s_uiQueue, &ev, portMAX_DELAY) == pdTRUE) {
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
                    // nothing here; main controls deep sleep
                    break;
            }
        }
    }
}

// -------------- EVENT HANDLERS ---------------

static void handle_button(const ButtonEvent& be)
{
    switch (s_state) {
        case UiState::MAIN:
        case UiState::RUNNING:
        case UiState::PAUSED:
        {
            if (be.id == ButtonId::UP) {
                // Start test
                EspNowProto::send_start();
            } else if (be.id == ButtonId::SELECT) {
                if (s_state == UiState::MAIN) {
                    s_state = UiState::SETTINGS;
                    draw_settings_screen();
                } else if (s_state == UiState::RUNNING) {
                    EspNowProto::send_pause();
                } else if (s_state == UiState::PAUSED) {
                    EspNowProto::send_resume();
                }
            } else if (be.id == ButtonId::DOWN) {
                // Stop needs confirmation
                s_state = UiState::CONFIRM_STOP;
                draw_confirm_stop();
            }
            break;
        }
        case UiState::CONFIRM_STOP:
            if (be.id == ButtonId::DOWN) {
                EspNowProto::send_stop();
                // Will transition to MAIN on STOP_ACK
            } else {
                // Any other button cancels stop
                s_state = UiState::MAIN;
                draw_main_screen();
            }
            break;
        case UiState::SETTINGS:
            if (be.id == ButtonId::SELECT) {
                SettingsStore::save(*s_settings);
                EspNowProto::send_config_set(*s_settings);
                s_state = UiState::MAIN;
                draw_main_screen();
            }
            break;
        case UiState::ERROR_SCREEN:
            s_errorBlink = false;
            s_state = UiState::MAIN;
            draw_main_screen();
            break;
        case UiState::COMPLETE:
            s_state = UiState::MAIN;
            draw_main_screen();
            break;
    }
}

static void handle_proto(const ProtoEvent& pe)
{
    switch (pe.type) {
        case ProtoEventType::CONFIG_UPDATED:
            *s_settings = pe.data.config;
            // Update display rotation if orientation changed
            if (g_display) {
                g_display->setRotation(s_settings->orientation_flipped ? 2 : 0);
            }
            draw_main_screen();
            break;
        case ProtoEventType::CONFIG_APPLY_OK:
            draw_main_screen();
            break;
        case ProtoEventType::CONFIG_APPLY_FAIL:
            s_errorCode = 1;
            s_state = UiState::ERROR_SCREEN;
            draw_error_screen();
            break;
        case ProtoEventType::STARTED:
            s_state = UiState::RUNNING;
            draw_main_screen();
            break;
        case ProtoEventType::PAUSED:
            s_state = UiState::PAUSED;
            draw_main_screen();
            break;
        case ProtoEventType::RESUMED:
            s_state = UiState::RUNNING;
            draw_main_screen();
            break;
        case ProtoEventType::STOPPED:
            s_state = UiState::MAIN;
            draw_main_screen();
            break;
        case ProtoEventType::STATUS:
            s_currentCycle = pe.data.status.cycle;
            update_status_footer();
            if (pe.data.status.state == TestState::ERROR) {
                s_errorCode = pe.data.status.err_code;
                s_state = UiState::ERROR_SCREEN;
                draw_error_screen();
            }
            break;
        case ProtoEventType::ERROR_EVENT:
            s_errorCode = pe.data.error.err_code;
            s_state = UiState::ERROR_SCREEN;
            draw_error_screen();
            break;
        case ProtoEventType::TEST_COMPLETED:
            s_state = UiState::COMPLETE;
            draw_complete_screen();
            break;
    }
}

// -------------- DISPLAY HELPER FUNCTIONS ---------------

static void set_title_style()
{
    if (!g_display) return;
    g_display->setTextColor(EPD_BLACK);
    g_display->setTextSize(2);
}

static void set_header_style()
{
    if (!g_display) return;
    g_display->setTextColor(EPD_BLACK);
    g_display->setTextSize(1);
}

static void set_button_style(bool selected)
{
    if (!g_display) return;
    g_display->setTextSize(2);
    if (selected) {
        g_display->setTextColor(EPD_RED);
    } else {
        g_display->setTextColor(EPD_BLACK);
    }
}

static void clear_screen()
{
    if (!g_display) return;
    g_display->clearBuffer();
}

// -------------- DRAWING FUNCTIONS ---------------

static void draw_main_screen()
{
    if (!g_display) {
        ESP_LOGW(TAG_UI, "Display not initialized, skipping draw");
        return;
    }

    clear_screen();

    // --- Title ---
    set_title_style();
    g_display->setCursor(10, 20);
    g_display->print("Fatigue Tester");

    // --- Settings Header (Top Bar) ---
    set_header_style();
    uint16_t y = 45;
    
    // Build header string with bounds method indicator
    String bounds_str = s_settings->bounds_method_stallguard ? "SG" : "ENC";
    String header = 
        "Cycles: " + String(s_settings->cycle_amount) +
        " | Time: " + String(s_settings->time_per_cycle) + "s" +
        " | Dwell: " + String(s_settings->dwell_time) + "s" +
        " | " + bounds_str;

    g_display->setCursor(4, y);
    g_display->print(header);

    // --- UI Controls ---
    y = 75;

    if (s_state == UiState::RUNNING) {
        // RUNNING STATE — show PAUSE / STOP
        set_button_style(false);
        g_display->setCursor(10, y);
        g_display->print("[RUNNING]");
        y += 22;

        set_button_style(true); 
        g_display->setCursor(15, y);
        g_display->print("PAUSE (SEL)");

    } else if (s_state == UiState::PAUSED) {
        set_button_style(false);
        g_display->setCursor(10, y);
        g_display->print("[PAUSED]");
        y += 22;

        set_button_style(true);
        g_display->setCursor(10, y);
        g_display->print("RESUME (UP)");

    } else {
        // IDLE / MAIN
        set_button_style(false);
        g_display->setCursor(10, y);
        g_display->print("Start: UP button");
        y += 22;

        g_display->setCursor(10, y);
        g_display->print("Settings: SEL");
    }

    // STOP (bottom small)
    set_button_style(false);
    g_display->setCursor(10, 122);
    g_display->print("Stop: DOWN");

    // Footer line will show cycle #
    update_status_footer();

    g_display->display(true);
}

static void draw_settings_screen()
{
    if (!g_display) {
        ESP_LOGW(TAG_UI, "Display not initialized, skipping draw");
        return;
    }

    clear_screen();

    set_title_style();
    g_display->setCursor(40, 20);
    g_display->print("Settings");

    set_header_style();

    uint16_t y = 50;

    g_display->setCursor(5, y);
    g_display->print("Cycle Amount: ");
    g_display->print(s_settings->cycle_amount);
    y += 18;

    g_display->setCursor(5, y);
    g_display->print("Time/Cycle: ");
    g_display->print(s_settings->time_per_cycle);
    g_display->print("s");
    y += 18;

    g_display->setCursor(5, y);
    g_display->print("Dwell Time: ");
    g_display->print(s_settings->dwell_time);
    g_display->print("s");
    y += 18;

    g_display->setCursor(5, y);
    g_display->print("Bounds Method: ");
    g_display->print(s_settings->bounds_method_stallguard ? "StallGuard" : "Encoder");
    y += 18;

    g_display->setCursor(5, y);
    g_display->print("Flip Orient: ");
    g_display->print(s_settings->orientation_flipped ? "ON" : "OFF");

    // Controls help bottom
    g_display->setCursor(5, 118);
    g_display->print("[SEL] Save  [DOWN] Back");

    g_display->display(true);
}

static void draw_confirm_stop()
{
    if (!g_display) {
        ESP_LOGW(TAG_UI, "Display not initialized, skipping draw");
        return;
    }

    clear_screen();

    set_title_style();
    g_display->setCursor(20, 20);
    g_display->print("Stop Test?");

    set_header_style();
    g_display->setCursor(20, 70);
    g_display->print("DOWN = Confirm STOP");
    g_display->setCursor(20, 92);
    g_display->print("Any other = Cancel");

    g_display->display(true);
}

static void draw_error_screen()
{
    if (!g_display) {
        ESP_LOGE(TAG_UI, "ERROR: code=%u (display not initialized)", s_errorCode);
        return;
    }

    clear_screen();
    g_display->fillRect(0, 0, 296, 128, EPD_WHITE);

    set_title_style();
    g_display->setCursor(40, 20);
    g_display->setTextColor(EPD_RED);
    g_display->print("ERROR");

    set_header_style();
    g_display->setTextColor(EPD_BLACK);
    g_display->setCursor(30, 60);
    g_display->print("Error Code: ");
    g_display->print(s_errorCode);

    g_display->setCursor(10, 90);
    g_display->print("Press any button...");

    g_display->display(true);

    // Optional blinking: flash red bar (slow for e-ink)
    // Note: E-ink should not refresh >1 Hz, so blinking is slow
    for (int i = 0; i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 second delay for e-ink
        
        if (i % 2 == 0) {
            // Flash red bar at top
            g_display->fillRect(0, 0, 296, 20, EPD_RED);
        } else {
            g_display->fillRect(0, 0, 296, 20, EPD_WHITE);
        }
        
        // Use partial update if available, otherwise full refresh
        #ifdef EPD_SUPPORTS_PARTIAL
        g_display->displayPartial(0, 296, 0, 20);
        #else
        g_display->display(true);
        #endif
    }

    // Redraw static error screen so user sees persistent state
    vTaskDelay(pdMS_TO_TICKS(500));
    clear_screen();
    g_display->fillRect(0, 0, 296, 128, EPD_WHITE);

    set_title_style();
    g_display->setCursor(40, 20);
    g_display->setTextColor(EPD_RED);
    g_display->print("ERROR");

    set_header_style();
    g_display->setTextColor(EPD_BLACK);
    g_display->setCursor(30, 60);
    g_display->print("Error Code: ");
    g_display->print(s_errorCode);

    g_display->setCursor(10, 90);
    g_display->print("Press any button...");

    g_display->display(true);
}

static void draw_complete_screen()
{
    if (!g_display) {
        ESP_LOGI(TAG_UI, "Test complete, cycle=%u (display not initialized)", s_currentCycle);
        return;
    }

    clear_screen();

    set_title_style();
    g_display->setCursor(10, 20);
    g_display->print("Test Complete!");

    set_header_style();
    g_display->setCursor(10, 60);
    g_display->print("Final Cycle #: ");
    g_display->print(s_currentCycle);

    g_display->setCursor(10, 90);
    g_display->print("Target: ");
    g_display->print(s_settings->cycle_amount);

    g_display->setCursor(10, 110);
    g_display->print("Press any button");

    g_display->display(true);
}

static void update_status_footer()
{
    if (!g_display) return;

    set_header_style();
    
    // Clear old footer area
    g_display->fillRect(0, 110, 296, 18, EPD_WHITE);

    g_display->setCursor(5, 122);
    if (s_state == UiState::RUNNING || s_state == UiState::PAUSED) {
        g_display->print("Cycle #: ");
        g_display->print(s_currentCycle);
        if (s_settings->cycle_amount > 0) {
            g_display->print("/");
            g_display->print(s_settings->cycle_amount);
        }
    } else if (s_state == UiState::MAIN) {
        g_display->print("Ready");
    } else {
        g_display->print("--");
    }

    // Use partial update if available, otherwise full refresh
    #ifdef EPD_SUPPORTS_PARTIAL
    g_display->displayPartial(110, 296, 18);
    #else
    // For drivers without partial update, we'll just update the whole screen
    // In practice, you might want to track if footer changed to avoid unnecessary refreshes
    #endif
}
