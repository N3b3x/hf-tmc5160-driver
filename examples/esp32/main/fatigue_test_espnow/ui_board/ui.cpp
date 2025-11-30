/**
 * @file ui.cpp
 * @brief UI implementation (stubbed for e-ink display integration)
 */

#include "ui.hpp"
#include "espnow_protocol.hpp"
#include "settings.hpp"
#include "esp_log.h"

// If you integrate Adafruit ThinkInk/EPD, include headers here:
// #include <Adafruit_GFX.h>
// #include <Adafruit_EPD.h>

static const char* TAG_UI = "UI";

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
static void draw_error_screen();
static void draw_complete_screen();
static void update_status_footer();

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

    // Here you would init display, set rotation based on settings->orientation_flipped
    // Example (pseudo):
    // display.begin();
    // display.setRotation(s_settings->orientation_flipped ? 2 : 0);

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
                draw_main_screen();
            }
            break;
        }
        case UiState::CONFIRM_STOP:
            if (be.id == ButtonId::DOWN) {
                EspNowProto::send_stop();
            } else {
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

// -------------- DRAWING HELPERS (STUBS) ---------------

static void draw_main_screen()
{
    // TODO: Replace with actual Adafruit EPD drawing
    ESP_LOGI(TAG_UI, "Draw main screen: cycles=%u, tper=%u, dwell=%u, bounds=%s",
             s_settings->cycle_amount,
             s_settings->time_per_cycle,
             s_settings->dwell_time,
             s_settings->bounds_method_stallguard ? "StallGuard" : "Encoder");
}

static void draw_settings_screen()
{
    ESP_LOGI(TAG_UI, "Draw settings screen");
}

static void draw_error_screen()
{
    ESP_LOGE(TAG_UI, "ERROR: code=%u", s_errorCode);
}

static void draw_complete_screen()
{
    ESP_LOGI(TAG_UI, "Draw complete screen, cycle=%u", s_currentCycle);
}

static void update_status_footer()
{
    ESP_LOGI(TAG_UI, "Update status footer: cycle=%u", s_currentCycle);
}
