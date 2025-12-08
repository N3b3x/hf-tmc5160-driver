/**
 * @file ui.cpp
 * @brief UI implementation with Adafruit 2.9" ThinkInk E-Ink tricolor display
 * 
 * Display: 2.9" ThinkInk FeatherWing (296x128 pixels horizontal)
 * Orientation: Vertical (128x296 pixels when rotated)
 * Buttons: UP (top), SELECT (middle), DOWN (bottom)
 * 
 * Features:
 * - Full menu navigation system
 * - Settings editing with value adjustment
 * - Proper e-ink tricolor support (black, white, red)
 * - Optimized refresh rates for e-ink
 */

#include "ui.hpp"
#include "espnow_protocol.hpp"
#include "settings.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "esp_timer.h"

// Adafruit E-Ink display libraries
#include "Adafruit_GFX.h"
#include "src/Adafruit_EPD.h"
#include "SPI.h"

static const char* TAG_UI = "UI";

// E-Ink: 2.9" ThinkInk FeatherWing (296 x 128 pixels horizontal)
// When rotated to portrait: 128 x 296 pixels (width x height)
// Uses IL0373 controller (standard for FeatherWing)
static Adafruit_IL0373* g_display = nullptr;

// Display dimensions (portrait mode after rotation)
static constexpr uint16_t DISPLAY_WIDTH = 128;   // Portrait width
static constexpr uint16_t DISPLAY_HEIGHT = 296;   // Portrait height

// Shared pointers
static QueueHandle_t s_uiQueue        = nullptr;
static Settings*     s_settings       = nullptr;
static uint32_t*     s_lastActivityTick = nullptr;

// State
static UiState   s_state       = UiState::MAIN;
static uint32_t  s_currentCycle= 0;
static bool      s_errorBlink  = false;
static uint8_t   s_errorCode   = 0;

// Settings menu navigation
static int s_settingsMenuIndex = 0;  // Current menu item index
static constexpr int SETTINGS_MENU_COUNT = 5;  // Number of settings items

// Value editing state
static uint32_t s_editValue = 0;
static uint32_t s_editMin = 0;
static uint32_t s_editMax = 0;
static uint32_t s_editStep = 1;

// Forward drawing helpers
static void draw_main_screen();
static void draw_settings_menu();
static void draw_settings_edit();
static void draw_confirm_stop();
static void draw_error_screen();
static void draw_complete_screen();
static void update_status_footer();

// Display helper functions
static void set_title_style();
static void set_header_style();
static void set_normal_style();
static void set_selected_style();
static void clear_screen();
static void draw_menu_item(const char* label, const char* value, int y, bool selected);
static void draw_edit_value(const char* label, uint32_t value, int y, bool selected);

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

    // CRITICAL: Initialize SPI bus BEFORE creating display
    // SPI pins are defined in config.hpp (SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN)
    SPI.begin(SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN);
    ESP_LOGI(TAG_UI, "SPI bus initialized: SCK=%d, MOSI=%d, MISO=%d", 
             SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN);

    // Initialize e-ink display for vertical/portrait (phone-like) orientation
    // 2.9" ThinkInk FeatherWing: Native 296x128 pixels (horizontal)
    // For phone-like vertical appearance: 128x296 pixels (portrait)
    // The display will use the SPI bus initialized above
    g_display = new Adafruit_IL0373(
        EINK_CS_PIN,
        EINK_DC_PIN,
        EINK_RESET_PIN,
        EINK_BUSY_PIN,
        296,   // width (native horizontal)
        128    // height (native horizontal)
    );

    g_display->begin();
    ESP_LOGI(TAG_UI, "E-ink display initialized (2.9\" ThinkInk, native 296x128)");

    // Set rotation for vertical/portrait phone-like appearance
    // Rotation 1 = 90° clockwise (portrait mode, 128x296 - phone-like)
    // Rotation 3 = 90° counter-clockwise (portrait mode flipped, 128x296)
    g_display->setRotation(s_settings->ui.orientation_flipped ? 3 : 1);
    ESP_LOGI(TAG_UI, "Display rotated to portrait/phone mode: %dx%d (width x height)", 
             DISPLAY_WIDTH, DISPLAY_HEIGHT);

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
        {
            if (be.id == ButtonId::UP) {
                // Start test
                EspNowProto::send_start();
            } else if (be.id == ButtonId::SELECT) {
                // Enter settings menu
                s_state = UiState::SETTINGS_MENU;
                s_settingsMenuIndex = 0;
                draw_settings_menu();
            } else if (be.id == ButtonId::DOWN) {
                // Stop needs confirmation
                s_state = UiState::CONFIRM_STOP;
                draw_confirm_stop();
            }
            break;
        }
        case UiState::RUNNING:
        {
            if (be.id == ButtonId::SELECT) {
                // Pause
                EspNowProto::send_pause();
            } else if (be.id == ButtonId::DOWN) {
                // Stop needs confirmation
                s_state = UiState::CONFIRM_STOP;
                draw_confirm_stop();
            }
            break;
        }
        case UiState::PAUSED:
        {
            if (be.id == ButtonId::UP) {
                // Resume
                EspNowProto::send_resume();
            } else if (be.id == ButtonId::SELECT) {
                // Resume (alternative)
                EspNowProto::send_resume();
            } else if (be.id == ButtonId::DOWN) {
                // Stop needs confirmation
                s_state = UiState::CONFIRM_STOP;
                draw_confirm_stop();
            }
            break;
        }
        case UiState::SETTINGS_MENU:
        {
            if (be.id == ButtonId::UP) {
                // Navigate up
                s_settingsMenuIndex = (s_settingsMenuIndex - 1 + SETTINGS_MENU_COUNT) % SETTINGS_MENU_COUNT;
                draw_settings_menu();
            } else if (be.id == ButtonId::DOWN) {
                // Navigate down
                s_settingsMenuIndex = (s_settingsMenuIndex + 1) % SETTINGS_MENU_COUNT;
                draw_settings_menu();
            } else if (be.id == ButtonId::SELECT) {
                // Enter edit mode for selected item
                switch (s_settingsMenuIndex) {
                    case 0: // Cycles
                        s_state = UiState::SETTINGS_EDIT_CYCLES;
                        s_editValue = s_settings->test_unit.cycle_amount;
                        s_editMin = 1;
                        s_editMax = 100000;
                        s_editStep = 100;
                        draw_settings_edit();
                        break;
                    case 1: // Time per cycle
                        s_state = UiState::SETTINGS_EDIT_TIME;
                        s_editValue = s_settings->test_unit.time_per_cycle;
                        s_editMin = 1;
                        s_editMax = 3600;
                        s_editStep = 1;
                        draw_settings_edit();
                        break;
                    case 2: // Dwell time
                        s_state = UiState::SETTINGS_EDIT_DWELL;
                        s_editValue = s_settings->test_unit.dwell_time;
                        s_editMin = 0;
                        s_editMax = 60;
                        s_editStep = 1;
                        draw_settings_edit();
                        break;
                    case 3: // Bounds method
                        s_state = UiState::SETTINGS_EDIT_METHOD;
                        s_editValue = s_settings->test_unit.bounds_method_stallguard ? 0 : 1;
                        s_editMin = 0;
                        s_editMax = 1;
                        s_editStep = 1;
                        draw_settings_edit();
                        break;
                    case 4: // Orientation
                        s_state = UiState::SETTINGS_EDIT_ORIENT;
                        s_editValue = s_settings->ui.orientation_flipped ? 1 : 0;
                        s_editMin = 0;
                        s_editMax = 1;
                        s_editStep = 1;
                        draw_settings_edit();
                        break;
                }
            }
            break;
        }
        case UiState::SETTINGS_EDIT_CYCLES:
        case UiState::SETTINGS_EDIT_TIME:
        case UiState::SETTINGS_EDIT_DWELL:
        case UiState::SETTINGS_EDIT_METHOD:
        case UiState::SETTINGS_EDIT_ORIENT:
        {
            if (be.id == ButtonId::UP) {
                // Increase value
                if (s_editValue + s_editStep <= s_editMax) {
                    s_editValue += s_editStep;
                } else {
                    s_editValue = s_editMax;
                }
                draw_settings_edit();
            } else if (be.id == ButtonId::DOWN) {
                // Decrease value
                if (s_editValue >= s_editStep) {
                    s_editValue -= s_editStep;
                } else {
                    s_editValue = s_editMin;
                }
                draw_settings_edit();
            } else if (be.id == ButtonId::SELECT) {
                // Save and return to settings menu
                switch (s_state) {
                    case UiState::SETTINGS_EDIT_CYCLES:
                        s_settings->test_unit.cycle_amount = s_editValue;
                        break;
                    case UiState::SETTINGS_EDIT_TIME:
                        s_settings->test_unit.time_per_cycle = s_editValue;
                        break;
                    case UiState::SETTINGS_EDIT_DWELL:
                        s_settings->test_unit.dwell_time = s_editValue;
                        break;
                    case UiState::SETTINGS_EDIT_METHOD:
                        s_settings->test_unit.bounds_method_stallguard = (s_editValue == 0);
                        break;
                    case UiState::SETTINGS_EDIT_ORIENT:
                        s_settings->ui.orientation_flipped = (s_editValue == 1);
                        if (g_display) {
                            g_display->setRotation(s_settings->ui.orientation_flipped ? 3 : 1);
                        }
                        break;
                    default:
                        break;
                }
                SettingsStore::save(*s_settings);
                EspNowProto::send_config_set(*s_settings);
                s_state = UiState::SETTINGS_MENU;
                draw_settings_menu();
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
                g_display->setRotation(s_settings->ui.orientation_flipped ? 3 : 1);
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
            if (s_state == UiState::RUNNING || s_state == UiState::PAUSED) {
                update_status_footer();
            }
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
        default:
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

static void set_normal_style()
{
    if (!g_display) return;
    g_display->setTextColor(EPD_BLACK);
    g_display->setTextSize(1);
}

static void set_selected_style()
{
    if (!g_display) return;
    g_display->setTextColor(EPD_RED);
    g_display->setTextSize(1);
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

    // Phone-like vertical layout: Title at top, content in middle, status at bottom
    
    // Title (Top, centered) - Phone header style
    set_title_style();
    g_display->setCursor(25, 8);
    g_display->print("Fatigue");
    g_display->setCursor(20, 28);
    g_display->print("Tester");

    // Divider line
    g_display->drawLine(0, 50, DISPLAY_WIDTH, 50, EPD_BLACK);

    // Settings Info (Compact, vertical layout - phone content area)
    set_header_style();
    uint16_t y = 60;
    
    // Build compact info strings
    const char* bounds_str = s_settings->test_unit.bounds_method_stallguard ? "StallGuard" : "Encoder";
    
    // Left-aligned labels, right-aligned values (phone-style)
    g_display->setCursor(5, y);
    g_display->print("Cycles:");
    char cycles_str[16];
    snprintf(cycles_str, sizeof(cycles_str), "%lu", s_settings->test_unit.cycle_amount);
    int16_t x1, y1;
    uint16_t w, h;
    g_display->getTextBounds(cycles_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor(DISPLAY_WIDTH - w - 5, y);
    g_display->print(cycles_str);
    y += 18;

    g_display->setCursor(5, y);
    g_display->print("Time/Cycle:");
    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%lus", s_settings->test_unit.time_per_cycle);
    g_display->getTextBounds(time_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor(DISPLAY_WIDTH - w - 5, y);
    g_display->print(time_str);
    y += 18;

    g_display->setCursor(5, y);
    g_display->print("Dwell:");
    char dwell_str[16];
    snprintf(dwell_str, sizeof(dwell_str), "%lus", s_settings->test_unit.dwell_time);
    g_display->getTextBounds(dwell_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor(DISPLAY_WIDTH - w - 5, y);
    g_display->print(dwell_str);
    y += 18;

    g_display->setCursor(5, y);
    g_display->print("Method:");
    g_display->getTextBounds(bounds_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor(DISPLAY_WIDTH - w - 5, y);
    g_display->print(bounds_str);
    y += 30;

    // Divider line before controls
    g_display->drawLine(0, y, DISPLAY_WIDTH, y, EPD_BLACK);
    y += 10;

    // State and Controls (Center area - phone action buttons)
    if (s_state == UiState::RUNNING) {
        set_normal_style();
        g_display->setCursor(35, y);
        g_display->print("[RUNNING]");
        y += 25;
        // Large button-style action
        g_display->fillRect(15, y - 5, DISPLAY_WIDTH - 30, 25, EPD_RED);
        set_normal_style();
        g_display->setTextColor(EPD_WHITE);
        g_display->setCursor(40, y + 5);
        g_display->print("PAUSE");
        g_display->setTextColor(EPD_BLACK);
    } else if (s_state == UiState::PAUSED) {
        set_normal_style();
        g_display->setCursor(35, y);
        g_display->print("[PAUSED]");
        y += 25;
        // Large button-style action
        g_display->fillRect(15, y - 5, DISPLAY_WIDTH - 30, 25, EPD_RED);
        set_normal_style();
        g_display->setTextColor(EPD_WHITE);
        g_display->setCursor(35, y + 5);
        g_display->print("RESUME");
        g_display->setTextColor(EPD_BLACK);
    } else {
        // Two button-style actions (phone-like)
        g_display->fillRect(15, y - 5, DISPLAY_WIDTH - 30, 22, EPD_BLACK);
        set_normal_style();
        g_display->setTextColor(EPD_WHITE);
        g_display->setCursor(40, y + 3);
        g_display->print("START");
        g_display->setTextColor(EPD_BLACK);
        y += 28;
        g_display->drawRect(15, y - 5, DISPLAY_WIDTH - 30, 22, EPD_BLACK);
        g_display->setCursor(42, y + 3);
        g_display->print("SETTINGS");
    }

    // Footer with cycle info (phone status bar style)
    update_status_footer();

    g_display->display(true);
}

static void draw_settings_menu()
{
    if (!g_display) {
        ESP_LOGW(TAG_UI, "Display not initialized, skipping draw");
        return;
    }

    clear_screen();

    // Phone-like header
    set_title_style();
    g_display->setCursor(30, 8);
    g_display->print("Settings");
    
    // Divider
    g_display->drawLine(0, 35, DISPLAY_WIDTH, 35, EPD_BLACK);

    set_header_style();
    uint16_t y = 45;

    // Draw menu items (phone list style)
    draw_menu_item("Cycles", "", y, s_settingsMenuIndex == 0);
    y += 24;
    draw_menu_item("Time/Cycle", "", y, s_settingsMenuIndex == 1);
    y += 24;
    draw_menu_item("Dwell", "", y, s_settingsMenuIndex == 2);
    y += 24;
    draw_menu_item("Method", "", y, s_settingsMenuIndex == 3);
    y += 24;
    draw_menu_item("Orientation", "", y, s_settingsMenuIndex == 4);

    // Help text at bottom (phone footer style)
    y = DISPLAY_HEIGHT - 35;
    g_display->drawLine(0, y - 5, DISPLAY_WIDTH, y - 5, EPD_BLACK);
    set_normal_style();
    g_display->setTextSize(1);
    g_display->setCursor(8, y);
    g_display->print("UP/DOWN: Navigate");
    y += 12;
    g_display->setCursor(8, y);
    g_display->print("SEL: Edit");

    g_display->display(true);
}

static void draw_menu_item(const char* label, const char* value, int y, bool selected)
{
    if (!g_display) return;
    
    // Phone-like list item with selection highlight
    if (selected) {
        // Draw selection indicator (red background - phone highlight style)
        g_display->fillRect(0, y - 4, DISPLAY_WIDTH, 22, EPD_RED);
        set_normal_style();
        g_display->setTextColor(EPD_WHITE);
    } else {
        set_normal_style();
        g_display->setTextColor(EPD_BLACK);
    }
    
    // Label on left
    g_display->setCursor(8, y);
    g_display->print(label);
    
    // Value on right (phone-style right-aligned)
    char value_str[32] = {0};
    switch (s_settingsMenuIndex) {
        case 0:
            snprintf(value_str, sizeof(value_str), "%lu", s_settings->cycle_amount);
            break;
        case 1:
            snprintf(value_str, sizeof(value_str), "%lus", s_settings->time_per_cycle);
            break;
        case 2:
            snprintf(value_str, sizeof(value_str), "%lus", s_settings->dwell_time);
            break;
        case 3:
            snprintf(value_str, sizeof(value_str), "%s", 
                     s_settings->test_unit.bounds_method_stallguard ? "StallGuard" : "Encoder");
            break;
        case 4:
            snprintf(value_str, sizeof(value_str), "%s", 
                     s_settings->ui.orientation_flipped ? "Flipped" : "Normal");
            break;
    }
    
    // Right-align value (phone-style)
    int16_t x1, y1;
    uint16_t w, h;
    g_display->getTextBounds(value_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor(DISPLAY_WIDTH - w - 8, y);
    g_display->print(value_str);
    
    // Show arrow indicator for selected item (phone-style)
    if (selected) {
        g_display->setTextColor(EPD_WHITE);
        g_display->setCursor(DISPLAY_WIDTH - 12, y);
        g_display->print(">");
    }
}

static void draw_settings_edit()
{
    if (!g_display) {
        ESP_LOGW(TAG_UI, "Display not initialized, skipping draw");
        return;
    }

    clear_screen();

    // Phone-like edit screen with large value display
    set_title_style();
    const char* label = "";
    switch (s_state) {
        case UiState::SETTINGS_EDIT_CYCLES:
            label = "Cycles";
            break;
        case UiState::SETTINGS_EDIT_TIME:
            label = "Time/Cycle";
            break;
        case UiState::SETTINGS_EDIT_DWELL:
            label = "Dwell Time";
            break;
        case UiState::SETTINGS_EDIT_METHOD:
            label = "Method";
            break;
        case UiState::SETTINGS_EDIT_ORIENT:
            label = "Orientation";
            break;
        default:
            label = "Edit";
            break;
    }
    
    // Header
    g_display->setCursor(25, 8);
    g_display->print(label);
    g_display->drawLine(0, 35, DISPLAY_WIDTH, 35, EPD_BLACK);

    // Large value display (phone number pad style)
    uint16_t y = 80;
    set_title_style();
    g_display->setTextSize(3);
    g_display->setTextColor(EPD_BLACK);
    
    // Center the value
    char value_str[32] = {0};
    if (s_state == UiState::SETTINGS_EDIT_METHOD) {
        snprintf(value_str, sizeof(value_str), "%s", s_editValue == 0 ? "StallGuard" : "Encoder");
    } else if (s_state == UiState::SETTINGS_EDIT_ORIENT) {
        snprintf(value_str, sizeof(value_str), "%s", s_editValue == 0 ? "Normal" : "Flipped");
    } else {
        snprintf(value_str, sizeof(value_str), "%lu", s_editValue);
        if (s_state == UiState::SETTINGS_EDIT_TIME || s_state == UiState::SETTINGS_EDIT_DWELL) {
            strcat(value_str, " s");
        }
    }
    
    int16_t x1, y1;
    uint16_t w, h;
    g_display->getTextBounds(value_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor((DISPLAY_WIDTH - w) / 2, y);
    g_display->print(value_str);

    // Range info (smaller, below value)
    y += 35;
    set_normal_style();
    g_display->setTextSize(1);
    g_display->setCursor(20, y);
    g_display->print("Range: ");
    g_display->print(s_editMin);
    g_display->print(" - ");
    g_display->print(s_editMax);

    // Help text at bottom (phone button style)
    y = DISPLAY_HEIGHT - 50;
    g_display->drawLine(0, y - 5, DISPLAY_WIDTH, y - 5, EPD_BLACK);
    set_normal_style();
    g_display->setCursor(15, y);
    g_display->print("UP: +");
    g_display->setCursor(60, y);
    g_display->print("DOWN: -");
    y += 15;
    g_display->setCursor(35, y);
    g_display->print("SEL: Save");

    g_display->display(true);
}

static void draw_confirm_stop()
{
    if (!g_display) {
        ESP_LOGW(TAG_UI, "Display not initialized, skipping draw");
        return;
    }

    clear_screen();

    // Phone-like confirmation dialog
    set_title_style();
    g_display->setCursor(40, 100);
    g_display->print("Stop");
    g_display->setCursor(35, 125);
    g_display->print("Test?");

    // Button-style actions at bottom
    uint16_t y = DISPLAY_HEIGHT - 60;
    g_display->drawLine(0, y, DISPLAY_WIDTH, y, EPD_BLACK);
    
    set_header_style();
    // Cancel button (left)
    g_display->drawRect(5, y + 10, 55, 30, EPD_BLACK);
    g_display->setCursor(18, y + 22);
    g_display->print("Cancel");
    
    // Confirm button (right, red)
    g_display->fillRect(68, y + 10, 55, 30, EPD_RED);
    g_display->setTextColor(EPD_WHITE);
    g_display->setCursor(80, y + 22);
    g_display->print("Stop");
    g_display->setTextColor(EPD_BLACK);

    g_display->display(true);
}

static void draw_error_screen()
{
    if (!g_display) {
        ESP_LOGE(TAG_UI, "ERROR: code=%u (display not initialized)", s_errorCode);
        return;
    }

    clear_screen();
    g_display->fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, EPD_WHITE);

    // Phone-like error dialog
    set_title_style();
    g_display->setCursor(35, 100);
    g_display->setTextColor(EPD_RED);
    g_display->print("ERROR");

    set_header_style();
    g_display->setTextColor(EPD_BLACK);
    g_display->setCursor(25, 135);
    g_display->print("Code: ");
    g_display->print(s_errorCode);

    // Button at bottom
    uint16_t y = DISPLAY_HEIGHT - 50;
    g_display->drawLine(0, y, DISPLAY_WIDTH, y, EPD_BLACK);
    g_display->fillRect(30, y + 10, DISPLAY_WIDTH - 60, 30, EPD_BLACK);
    g_display->setTextColor(EPD_WHITE);
    g_display->setCursor(45, y + 22);
    g_display->print("OK");
    g_display->setTextColor(EPD_BLACK);

    g_display->display(true);
}

static void draw_complete_screen()
{
    if (!g_display) {
        ESP_LOGI(TAG_UI, "Test complete, cycle=%u (display not initialized)", s_currentCycle);
        return;
    }

    clear_screen();

    // Phone-like completion screen
    set_title_style();
    g_display->setCursor(20, 100);
    g_display->print("Complete!");

    set_header_style();
    char cycles_str[32];
    snprintf(cycles_str, sizeof(cycles_str), "%lu/%lu", s_currentCycle, s_settings->cycle_amount);
    int16_t x1, y1;
    uint16_t w, h;
    g_display->getTextBounds(cycles_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor((DISPLAY_WIDTH - w) / 2, 135);
    g_display->print("Cycles: ");
    g_display->print(cycles_str);

    // Button at bottom
    uint16_t y = DISPLAY_HEIGHT - 50;
    g_display->drawLine(0, y, DISPLAY_WIDTH, y, EPD_BLACK);
    g_display->fillRect(30, y + 10, DISPLAY_WIDTH - 60, 30, EPD_BLACK);
    g_display->setTextColor(EPD_WHITE);
    g_display->setCursor(50, y + 22);
    g_display->print("OK");
    g_display->setTextColor(EPD_BLACK);

    g_display->display(true);
}

static void update_status_footer()
{
    if (!g_display) return;

    set_header_style();
    
    // Phone status bar style footer (bottom of vertical display)
    g_display->fillRect(0, DISPLAY_HEIGHT - 22, DISPLAY_WIDTH, 22, EPD_BLACK);
    g_display->setTextColor(EPD_WHITE);
    
    // Center the status text
    char status_str[32] = {0};
    if (s_state == UiState::RUNNING || s_state == UiState::PAUSED) {
        snprintf(status_str, sizeof(status_str), "Cycle: %lu", s_currentCycle);
        if (s_settings->cycle_amount > 0) {
            char temp[32];
            snprintf(temp, sizeof(temp), "/%lu", s_settings->cycle_amount);
            strcat(status_str, temp);
        }
    } else if (s_state == UiState::MAIN) {
        strcpy(status_str, "Ready");
    } else {
        strcpy(status_str, "--");
    }
    
    int16_t x1, y1;
    uint16_t w, h;
    g_display->getTextBounds(status_str, 0, 0, &x1, &y1, &w, &h);
    g_display->setCursor((DISPLAY_WIDTH - w) / 2, DISPLAY_HEIGHT - 12);
    g_display->print(status_str);
    g_display->setTextColor(EPD_BLACK);

    // Note: For e-ink, we avoid partial updates unless necessary
    // Full refresh is acceptable for footer updates
}
