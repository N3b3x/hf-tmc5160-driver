/**
 * @file oled_menu.hpp
 * @brief Non-blocking menu system for OLED display with rotary encoder
 * 
 * Features:
 * - Hierarchical menu structure
 * - Rotary encoder navigation
 * - Button actions (BACK, CONFIRM, encoder button)
 * - Settings editing with value adjustment
 * - Compatible with existing Settings structure
 */

#pragma once

#include "../espnow_protocol.hpp"
#include "settings.hpp"
#include "../../components/Adafruit_SH1106_ESPIDF/Adafruit_SH1106.h"
#include "../../components/EC11_Encoder/inc/ec11_encoder.hpp"
#include "button.hpp"
#include <cstdint>
#include <cstddef>

/**
 * @brief Menu item types
 */
enum class MenuItemType {
    SUBMENU,        // Opens a submenu
    ACTION,         // Executes an action
    VALUE_EDIT,     // Edits a numeric value
    TOGGLE,         // Toggles a boolean value
    CHOICE          // Selects from a list of options (boolean for now)
};

/**
 * @brief Menu item structure
 */
struct MenuItem {
    const char* label;          // Display label
    MenuItemType type;          // Item type
    void* data;                  // Type-specific data
    MenuItem* parent;            // Parent menu (for navigation)
    MenuItem* children;          // Child items (for submenus)
    size_t child_count;          // Number of children
    const char* units;           // Units string (e.g., "sec", "cycles")
};

/**
 * @brief Menu system class
 */
class OLEDMenu {
public:
    /**
     * @brief Constructor
     * @param display SH1106 display instance
     * @param encoder EC11 encoder instance
     * @param settings Settings structure pointer
     */
    OLEDMenu(Adafruit_SH1106* display, EC11Encoder* encoder, Settings* settings);
    
    /**
     * @brief Initialize menu system
     * @return true if successful
     */
    bool begin();
    
    /**
     * @brief Update menu (call from main loop)
     * @return true if display needs refresh
     */
    bool update();
    
    /**
     * @brief Process button event
     * @param btn_id Button ID
     * @return true if event was handled within menu, false if menu exited
     */
    bool handleButton(ButtonId btn_id);
    
    /**
     * @brief Process encoder rotation
     * @param direction Rotation direction
     */
    void handleEncoderRotation(EC11Encoder::Direction direction);
    
    /**
     * @brief Process encoder button
     * @param pressed Button state
     */
    void handleEncoderButton(bool pressed);
    
    /**
     * @brief Force menu refresh
     */
    void refresh();

    /**
     * @brief Reset input state (e.g. when entering menu)
     */
    void resetInputState();

    /**
     * @brief Check if menu exit was requested (e.g. via Back item)
     * @return true if exit requested
     */
    bool isExitRequested() const { return exit_requested_; }

private:
    Adafruit_SH1106* display_;
    EC11Encoder* encoder_;
    Settings* settings_;
    
    bool last_button_state_; // Track encoder button state
    int32_t last_encoder_pos_;  // Track encoder position between updates
    MenuItem* current_menu_;
    int current_index_;
    bool needs_refresh_;
    bool exit_requested_; // Track exit request
    
    // Value editing state
    bool editing_value_;
    bool editing_choice_; // New state for choice editing
    uint32_t* edit_value_ptr_;
    bool* edit_bool_ptr_; // Pointer for boolean choice editing
    uint32_t edit_min_;
    uint32_t edit_max_;
    uint32_t edit_step_;
    
    // Menu structure
    MenuItem root_menu_;
    MenuItem settings_menu_;
    MenuItem cycles_item_;
    MenuItem time_item_;
    MenuItem dwell_item_;
    MenuItem orient_item_;
    MenuItem bounds_item_;
    MenuItem back_item_; // Generic back item
    MenuItem root_back_item_; // Back item for root menu
    
    // Menu building
    void buildMenuStructure();
    
    // Rendering
    void renderMenu();
    void renderValueEdit();
    void renderChoiceEdit();
    void renderMainScreen();
    
    // Navigation
    void navigateUp();
    void navigateDown();
    bool enterItem();
    void exitMenu();
    
    // Value editing
    void startValueEdit(uint32_t* value_ptr, uint32_t min_val, uint32_t max_val, uint32_t step);
    void adjustValue(int32_t delta);
    void saveValue();
    void cancelValueEdit();
};

