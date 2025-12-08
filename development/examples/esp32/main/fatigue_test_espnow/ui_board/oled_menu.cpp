/**
 * @file oled_menu.cpp
 * @brief OLED menu system implementation
 */

#include "oled_menu.hpp"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG_MENU = "OLEDMenu";

OLEDMenu::OLEDMenu(Adafruit_SH1106* display, EC11Encoder* encoder, Settings* settings)
    : display_(display), encoder_(encoder), settings_(settings), last_button_state_(false),
      last_encoder_pos_(0), current_menu_(nullptr), current_index_(0), needs_refresh_(true),
      exit_requested_(false),
      editing_value_(false), editing_choice_(false), edit_value_ptr_(nullptr), edit_bool_ptr_(nullptr),
      edit_min_(0), edit_max_(0), edit_step_(1) {
    buildMenuStructure();
}

bool OLEDMenu::begin() {
    if (!display_ || !encoder_ || !settings_) {
        ESP_LOGE(TAG_MENU, "Invalid parameters");
        return false;
    }
    
    current_menu_ = &root_menu_;
    current_index_ = 0;
    needs_refresh_ = true;
    exit_requested_ = false;
    
    // CRITICAL: Initialize last_encoder_pos_ from actual encoder position
    // This prevents the "always down" bug at startup
    last_encoder_pos_ = encoder_->getPosition() / 4;
    ESP_LOGI(TAG_MENU, "Initialized encoder position tracking: %ld", (long)last_encoder_pos_);
    
    // Apply initial rotation based on settings
    if (settings_->ui.orientation_flipped) {
        display_->setRotation(2); // 180 degrees
        ESP_LOGI(TAG_MENU, "Display rotation set to 180 degrees (flipped)");
    } else {
        display_->setRotation(0); // 0 degrees
        ESP_LOGI(TAG_MENU, "Display rotation set to 0 degrees (normal)");
    }
    
    ESP_LOGI(TAG_MENU, "Menu system initialized (orientation_flipped=%d)", settings_->ui.orientation_flipped);
    // Brief hint: show orientation status in logs; UI footer already indicates MENU/START
    return true;
}

bool OLEDMenu::update() {
    // Poll encoder position changes
    // Divide by 4 to handle standard EC11 4-pulses-per-detent resolution
    // This prevents "jumping" multiple items per click
    int32_t current_pos = encoder_->getPosition() / 4;
    
    if (current_pos != last_encoder_pos_) {
        ESP_LOGI(TAG_MENU, "Encoder rotated: %ld -> %ld (flipped=%d)", 
                 (long)last_encoder_pos_, (long)current_pos, settings_->ui.orientation_flipped);
        
        // Determine direction (default: pos decrease = CW)
        bool is_cw = (current_pos < last_encoder_pos_);
        
        // REMOVED: Do not invert direction if screen is flipped.
        // CW rotation should always mean "Next" or "Increase", regardless of screen orientation.
        // if (settings_->ui.orientation_flipped) {
        //    is_cw = !is_cw;
        // }
        
        if (is_cw) {
            handleEncoderRotation(EC11Encoder::Direction::CW);
        } else {
            handleEncoderRotation(EC11Encoder::Direction::CCW);
        }
        last_encoder_pos_ = current_pos;
    }
    
    // Check encoder button state changes
    bool current_button = encoder_->isButtonPressed();
    if (current_button && !last_button_state_) {
        ESP_LOGI(TAG_MENU, "Encoder button pressed");
        handleEncoderButton(true);
    } else if (!current_button && last_button_state_) {
        handleEncoderButton(false);
    }
    last_button_state_ = current_button;
    
    // Refresh display if needed
    if (needs_refresh_) {
        if (editing_value_) {
            renderValueEdit();
        } else if (editing_choice_) {
            renderChoiceEdit();
        } else {
            renderMenu();
        }
        needs_refresh_ = false;
        return true;
    }
    
    return false;
}

bool OLEDMenu::handleButton(ButtonId btn_id) {
    ESP_LOGI(TAG_MENU, "Menu Button event: %d", (int)btn_id);
    if (editing_value_) {
        if (btn_id == ButtonId::BACK) {
            cancelValueEdit();
        } else if (btn_id == ButtonId::CONFIRM) {
            saveValue();
        }
        needs_refresh_ = true;
        return true;
    } else if (editing_choice_) {
        if (btn_id == ButtonId::BACK) {
            editing_choice_ = false;
            edit_bool_ptr_ = nullptr;
        } else if (btn_id == ButtonId::CONFIRM) {
        if (edit_bool_ptr_) {
            *edit_bool_ptr_ = (bool)edit_min_;
            // Handle side effects
            if (settings_->ui.orientation_flipped) {
                ESP_LOGI(TAG_MENU, "Flipping screen (180)");
                display_->setRotation(2); // 180 degrees
            } else {
                ESP_LOGI(TAG_MENU, "Un-flipping screen (0)");
                display_->setRotation(0); // 0 degrees
            }
                // Force clear to prevent artifacts
                display_->clearDisplay();
                display_->display();
            }
            editing_choice_ = false;
            edit_bool_ptr_ = nullptr;
        }
        needs_refresh_ = true;
        return true;
    } else {
        if (btn_id == ButtonId::BACK) {
            if (current_menu_->parent == nullptr) {
                return false; // Exit menu system
            }
            exitMenu();
        } else if (btn_id == ButtonId::CONFIRM) {
            if (!enterItem()) {
                return false; // Exit menu system
            }
        }
        needs_refresh_ = true;
        return true;
    }
}

void OLEDMenu::handleEncoderRotation(EC11Encoder::Direction direction) {
    if (editing_value_) {
        if (direction == EC11Encoder::Direction::CW) {
            adjustValue(edit_step_);
        } else if (direction == EC11Encoder::Direction::CCW) {
            adjustValue(-static_cast<int32_t>(edit_step_));
        }
        needs_refresh_ = true;
    } else if (editing_choice_) {
        // Toggle selection
        edit_min_ = !edit_min_;
        needs_refresh_ = true;
    } else {
        if (direction == EC11Encoder::Direction::CW) {
            navigateDown();
        } else if (direction == EC11Encoder::Direction::CCW) {
            navigateUp();
        }
        
        // Log current selection for debugging menu navigation
        if (current_menu_ && current_menu_->children && current_index_ < static_cast<int>(current_menu_->child_count)) {
            MenuItem* item = &current_menu_->children[current_index_];
            ESP_LOGI(TAG_MENU, "Menu nav: encoder_pos=%ld, index=%d, item='%s'", 
                     (long)last_encoder_pos_, current_index_, item->label ? item->label : "?");
        }
        
        needs_refresh_ = true;
    }
}

void OLEDMenu::handleEncoderButton(bool pressed) {
    if (pressed && !editing_value_ && !editing_choice_) {
        enterItem();
        needs_refresh_ = true;
    } else if (pressed && editing_value_) {
        saveValue();
        needs_refresh_ = true;
    } else if (pressed && editing_choice_) {
        // Confirm choice
        if (edit_bool_ptr_) {
            *edit_bool_ptr_ = (bool)edit_min_;
            // Handle side effects
            if (settings_->ui.orientation_flipped) {
                ESP_LOGI(TAG_MENU, "Flipping screen (180)");
                display_->setRotation(2); // 180 degrees
            } else {
                ESP_LOGI(TAG_MENU, "Un-flipping screen (0)");
                display_->setRotation(0); // 0 degrees
            }
            // Force clear to prevent artifacts
            display_->clearDisplay();
            display_->display();
        }
        editing_choice_ = false;
        edit_bool_ptr_ = nullptr;
        needs_refresh_ = true;
    }
}

void OLEDMenu::refresh() {
    needs_refresh_ = true;
}

void OLEDMenu::buildMenuStructure() {
    // Settings items
    cycles_item_ = {
        .label = "Cycles",
        .type = MenuItemType::VALUE_EDIT,
        .data = &settings_->test_unit.cycle_amount,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0,
        .units = ""
    };
    
    time_item_ = {
        .label = "Time/Cycle",
        .type = MenuItemType::VALUE_EDIT,
        .data = &settings_->test_unit.time_per_cycle,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0,
        .units = "s"
    };
    
    dwell_item_ = {
        .label = "Dwell Time",
        .type = MenuItemType::VALUE_EDIT,
        .data = &settings_->test_unit.dwell_time,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0,
        .units = "s"
    };
    
    orient_item_ = {
        .label = "Flip Screen",
        .type = MenuItemType::CHOICE,
        .data = &settings_->ui.orientation_flipped,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0,
        .units = ""
    };
    
    bounds_item_ = {
        .label = "Bounds Mode",
        .type = MenuItemType::CHOICE,
        .data = &settings_->test_unit.bounds_method_stallguard,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0,
        .units = ""
    };

    back_item_ = {
        .label = "Back",
        .type = MenuItemType::ACTION,
        .data = nullptr,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0,
        .units = ""
    };
    
    // Link children to settings menu
    // The MenuItem struct expects `MenuItem* children` to point to an array of MenuItem objects.
    // Since we defined individual MenuItem members, we need to link them.
    // However, the current struct definition `MenuItem* children` implies a contiguous array if we index it like `children[i]`.
    // So we must allocate a contiguous array for the children.
    
    // Allocate children array for root menu
    static MenuItem root_children[2]; // Will be populated later
    
    // Allocate children array for settings menu
    static MenuItem settings_children[6];
    
    // Populate settings children
    settings_children[0] = cycles_item_;
    settings_children[1] = time_item_;
    settings_children[2] = dwell_item_;
    settings_children[3] = bounds_item_;
    settings_children[4] = orient_item_;
    settings_children[5] = back_item_;
    
    // Settings menu
    settings_menu_ = {
        .label = "Settings",
        .type = MenuItemType::SUBMENU,
        .data = nullptr,
        .parent = &root_menu_,
        .children = settings_children,
        .child_count = 6,
        .units = ""
    };
    
    root_back_item_ = {
        .label = "Back",
        .type = MenuItemType::ACTION,
        .data = nullptr,
        .parent = &root_menu_,
        .children = nullptr,
        .child_count = 0,
        .units = ""
    };

    // Populate root children
    root_children[0] = settings_menu_;
    root_children[1] = root_back_item_;
    
    // Root menu
    root_menu_ = {
        .label = "Main",
        .type = MenuItemType::SUBMENU,
        .data = nullptr,
        .parent = nullptr,
        .children = root_children,
        .child_count = 2,
        .units = ""
    };
}

void OLEDMenu::renderMenu() {
    if (!display_ || !current_menu_) {
        return;
    }
    
    display_->clearDisplay();
    
    // Draw title
    display_->setTextSize(1);
    display_->setTextColor(1);
    display_->setCursor(0, 0);
    display_->print(current_menu_->label);
    display_->drawLine(0, 9, 128, 9, 1);
    
    // Draw menu items
    if (current_menu_->children && current_menu_->child_count > 0) {
        int start_idx = (current_index_ > 3) ? current_index_ - 3 : 0;
        int end_idx = start_idx + 4;
        if (end_idx > static_cast<int>(current_menu_->child_count)) {
            end_idx = current_menu_->child_count;
            start_idx = end_idx - 4;
            if (start_idx < 0) start_idx = 0;
        }
        
        int y = 12;
        for (int i = start_idx; i < end_idx; i++) {
            MenuItem* item = &current_menu_->children[i];
            bool selected = (i == current_index_);
            
            if (selected) {
                display_->fillRect(0, y - 1, 128, 11, 1);
                display_->setTextColor(0);
            } else {
                display_->setTextColor(1);
            }
            
            display_->setCursor(2, y);
            display_->print(item->label);
            
            // Display value if applicable
            if (item->type == MenuItemType::VALUE_EDIT && item->data) {
                char buf[16];
                snprintf(buf, sizeof(buf), "[%lu%s]", *(uint32_t*)item->data, item->units ? item->units : "");
                
                // Right align value
                int16_t x1, y1;
                uint16_t w, h;
                display_->getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
                display_->setCursor(126 - w, y);
                display_->print(buf);
            } else if (item->type == MenuItemType::TOGGLE && item->data) {
                bool val = *(uint8_t*)item->data;
                const char* str = val ? "[ON]" : "[OFF]";
                
                int16_t x1, y1;
                uint16_t w, h;
                display_->getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
                display_->setCursor(126 - w, y);
                display_->print(str);
            } else if (item->type == MenuItemType::CHOICE && item->data) {
                bool val = *(bool*)item->data;
                const char* str = "[?]";
                if (strcmp(item->label, "Bounds Mode") == 0) {
                    str = val ? "[STALL]" : "[ENC]";
                } else if (strcmp(item->label, "Flip Screen") == 0) {
                    str = val ? "[FLIP]" : "[NORM]";
                }
                
                int16_t x1, y1;
                uint16_t w, h;
                display_->getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
                display_->setCursor(126 - w, y);
                display_->print(str);
            }
            
            y += 12;
        }
    }
    
    display_->display();
}

void OLEDMenu::renderChoiceEdit() {
    if (!display_ || !edit_bool_ptr_) {
        return;
    }
    
    display_->clearDisplay();
    
    // Draw title
    display_->setTextSize(1);
    display_->setTextColor(1);
    display_->setCursor(0, 0);
    
    // Find the item to get label
    const char* label = "Edit Choice";
    if (current_menu_ && current_menu_->children) {
        MenuItem* item = &current_menu_->children[current_index_];
        if (item->label) label = item->label;
    }
    display_->print(label);
    display_->drawLine(0, 9, 128, 9, 1);
    
    // Draw choices
    display_->setTextSize(1);
    
    const char* opt1 = "Option 1";
    const char* opt2 = "Option 2";
    
    if (strcmp(label, "Bounds Mode") == 0) {
        opt1 = " [ENC] ";
        opt2 = "[STALL]";
    } else if (strcmp(label, "Flip Screen") == 0) {
        opt1 = "[NORM]";
        opt2 = "[FLIP]";
    }
    
    // edit_min_ holds the temp value (0 or 1)
    bool val = (bool)edit_min_;
    ESP_LOGI(TAG_MENU, "Rendering choice: %s, edit_min_=%lu, val=%d", label, edit_min_, val);
    
    // Draw Option 1 (False)
    if (!val) {
        display_->setTextColor(0, 1); // Inverted
    } else {
        display_->setTextColor(1, 0);
    }
    display_->setCursor(10, 30);
    display_->print(opt1);
    
    // Draw Option 2 (True)
    if (val) {
        display_->setTextColor(0, 1); // Inverted
    } else {
        display_->setTextColor(1, 0);
    }
    display_->setCursor(70, 30);
    display_->print(opt2);
    
    // Reset color
    display_->setTextColor(1);
    
    // Draw help
    display_->setCursor(0, 54);
    display_->print("Rotate: Sel  Push: OK");
    
    display_->display();
}

void OLEDMenu::renderValueEdit() {
    if (!display_ || !edit_value_ptr_) {
        return;
    }
    
    display_->clearDisplay();
    
    // Draw title
    display_->setTextSize(1);
    display_->setTextColor(1);
    display_->setCursor(0, 0);
    
    // Find the item to get label
    const char* label = "Edit Value";
    if (current_menu_ && current_menu_->children) {
        MenuItem* item = &current_menu_->children[current_index_];
        if (item->label) label = item->label;
    }
    
    display_->print(label);
    display_->drawLine(0, 9, 128, 9, 1);
    
    // Draw value (large)
    display_->setTextSize(2);
    display_->setCursor(10, 25);
    char buf[32];
    // Find the item to get units
    const char* units = "";
    if (current_menu_ && current_menu_->children) {
        MenuItem* item = &current_menu_->children[current_index_];
        if (item->units) units = item->units;
    }
    
    snprintf(buf, sizeof(buf), "%lu %s", *edit_value_ptr_, units);
    display_->print(buf);
    
    // Draw help
    display_->setTextSize(1);
    display_->setCursor(0, 54);
    display_->print("Rotate: +/-  Push: OK");
    
    display_->display();
}

void OLEDMenu::navigateUp() {
    if (current_menu_->child_count > 0) {
        current_index_ = (current_index_ - 1 + current_menu_->child_count) % current_menu_->child_count;
    }
}

void OLEDMenu::navigateDown() {
    if (current_menu_->child_count > 0) {
        current_index_ = (current_index_ + 1) % current_menu_->child_count;
    }
}

bool OLEDMenu::enterItem() {
    if (!current_menu_->children || current_index_ >= static_cast<int>(current_menu_->child_count)) {
        return true;
    }
    
    MenuItem* item = &current_menu_->children[current_index_];
    
    if (item->type == MenuItemType::SUBMENU && item->children) {
        current_menu_ = item;
        current_index_ = 0;
    } else if (item->type == MenuItemType::VALUE_EDIT) {
        uint32_t* value_ptr = static_cast<uint32_t*>(item->data);
        if (strcmp(item->label, "Cycles") == 0) {
            startValueEdit(value_ptr, 1, 100000, 100);
        } else if (strcmp(item->label, "Time/Cycle") == 0) {
            startValueEdit(value_ptr, 1, 3600, 1);
        } else if (strcmp(item->label, "Dwell Time") == 0) {
            startValueEdit(value_ptr, 0, 60, 1);
        }
    } else if (item->type == MenuItemType::TOGGLE) {
        bool* value_ptr = static_cast<bool*>(item->data);
        *value_ptr = !(*value_ptr);
    } else if (item->type == MenuItemType::CHOICE) {
        bool* value_ptr = static_cast<bool*>(item->data);
        editing_choice_ = true;
        edit_bool_ptr_ = value_ptr;
        edit_min_ = (uint32_t)(*value_ptr); // Use edit_min_ as temp storage
        ESP_LOGI(TAG_MENU, "Entering choice edit: %s, current value=%d, edit_min_=%lu", 
                 item->label, *value_ptr, edit_min_);
    } else if (item->type == MenuItemType::ACTION) {
        ESP_LOGI(TAG_MENU, "Action: %s", item->label);
        if (strcmp(item->label, "Back") == 0) {
            // Check if we are at root menu (either by pointer or parent)
            if (current_menu_ == &root_menu_ || current_menu_->parent == nullptr) {
                ESP_LOGI(TAG_MENU, "Root Back -> Exit");
                exit_requested_ = true;
                return false; // Exit menu system
            }
            ESP_LOGI(TAG_MENU, "Submenu Back -> Up");
            exitMenu();
        }
    }
    return true;
}

void OLEDMenu::exitMenu() {
    if (current_menu_->parent) {
        current_menu_ = current_menu_->parent;
        current_index_ = 0;
    }
}

void OLEDMenu::startValueEdit(uint32_t* value_ptr, uint32_t min_val, uint32_t max_val, uint32_t step) {
    editing_value_ = true;
    edit_value_ptr_ = value_ptr;
    edit_min_ = min_val;
    edit_max_ = max_val;
    edit_step_ = step;
}

void OLEDMenu::adjustValue(int32_t delta) {
    if (!edit_value_ptr_) return;
    
    int32_t new_value = static_cast<int32_t>(*edit_value_ptr_) + delta;
    if (new_value < static_cast<int32_t>(edit_min_)) {
        new_value = edit_min_;
    } else if (new_value > static_cast<int32_t>(edit_max_)) {
        new_value = edit_max_;
    }
    
    *edit_value_ptr_ = static_cast<uint32_t>(new_value);
}

void OLEDMenu::saveValue() {
    editing_value_ = false;
    edit_value_ptr_ = nullptr;
    // Settings will be saved by the main application
}

void OLEDMenu::cancelValueEdit() {
    editing_value_ = false;
    edit_value_ptr_ = nullptr;
}

void OLEDMenu::resetInputState() {
    exit_requested_ = false;
    if (encoder_) {
        last_button_state_ = encoder_->isButtonPressed();
        // CRITICAL: Reset encoder position tracking to prevent stale state
        // This ensures direction detection works correctly when entering settings
        last_encoder_pos_ = encoder_->getPosition() / 4;
    }
}

