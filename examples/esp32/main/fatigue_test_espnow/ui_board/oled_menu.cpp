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
    : display_(display), encoder_(encoder), settings_(settings),
      current_menu_(nullptr), current_index_(0), needs_refresh_(true),
      editing_value_(false), edit_value_ptr_(nullptr),
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
    
    ESP_LOGI(TAG_MENU, "Menu system initialized");
    return true;
}

bool OLEDMenu::update() {
    // Poll encoder position changes
    static int32_t last_pos = encoder_->getPosition();
    int32_t current_pos = encoder_->getPosition();
    if (current_pos != last_pos) {
        ESP_LOGI(TAG_MENU, "Encoder rotated: %ld -> %ld", (long)last_pos, (long)current_pos);
        if (current_pos > last_pos) {
            handleEncoderRotation(EC11Encoder::Direction::CW);
        } else {
            handleEncoderRotation(EC11Encoder::Direction::CCW);
        }
        last_pos = current_pos;
    }
    
    // Check encoder button state changes
    static bool last_button = encoder_->isButtonPressed();
    bool current_button = encoder_->isButtonPressed();
    if (current_button && !last_button) {
        ESP_LOGI(TAG_MENU, "Encoder button pressed");
        handleEncoderButton(true);
    } else if (!current_button && last_button) {
        handleEncoderButton(false);
    }
    last_button = current_button;
    
    // Refresh display if needed
    if (needs_refresh_) {
        if (editing_value_) {
            renderValueEdit();
        } else {
            renderMenu();
        }
        needs_refresh_ = false;
        return true;
    }
    
    return false;
}

void OLEDMenu::handleButton(ButtonId btn_id) {
    ESP_LOGI(TAG_MENU, "Menu Button event: %d", (int)btn_id);
    if (editing_value_) {
        if (btn_id == ButtonId::BACK) {
            cancelValueEdit();
        } else if (btn_id == ButtonId::CONFIRM) {
            saveValue();
        }
    } else {
        if (btn_id == ButtonId::BACK) {
            exitMenu();
        } else if (btn_id == ButtonId::CONFIRM) {
            enterItem();
        }
    }
    needs_refresh_ = true;
}

void OLEDMenu::handleEncoderRotation(EC11Encoder::Direction direction) {
    if (editing_value_) {
        if (direction == EC11Encoder::Direction::CW) {
            adjustValue(edit_step_);
        } else if (direction == EC11Encoder::Direction::CCW) {
            adjustValue(-static_cast<int32_t>(edit_step_));
        }
        needs_refresh_ = true;
    } else {
        if (direction == EC11Encoder::Direction::CW) {
            navigateDown();
        } else if (direction == EC11Encoder::Direction::CCW) {
            navigateUp();
        }
        needs_refresh_ = true;
    }
}

void OLEDMenu::handleEncoderButton(bool pressed) {
    if (pressed && !editing_value_) {
        enterItem();
        needs_refresh_ = true;
    } else if (pressed && editing_value_) {
        saveValue();
        needs_refresh_ = true;
    }
}

void OLEDMenu::refresh() {
    needs_refresh_ = true;
}

void OLEDMenu::buildMenuStructure() {
    // Root menu
    root_menu_ = {
        .label = "Main",
        .type = MenuItemType::SUBMENU,
        .data = nullptr,
        .parent = nullptr,
        .children = &settings_menu_,
        .child_count = 1
    };
    
    // Settings menu
    settings_menu_ = {
        .label = "Settings",
        .type = MenuItemType::SUBMENU,
        .data = nullptr,
        .parent = &root_menu_,
        .children = &cycles_item_,
        .child_count = 5
    };
    
    // Settings items
    cycles_item_ = {
        .label = "Cycles",
        .type = MenuItemType::VALUE_EDIT,
        .data = &settings_->cycle_amount,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0
    };
    
    time_item_ = {
        .label = "Time/Cycle",
        .type = MenuItemType::VALUE_EDIT,
        .data = &settings_->time_per_cycle,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0
    };
    
    dwell_item_ = {
        .label = "Dwell",
        .type = MenuItemType::VALUE_EDIT,
        .data = &settings_->dwell_time,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0
    };
    
    method_item_ = {
        .label = "Method",
        .type = MenuItemType::TOGGLE,
        .data = &settings_->bounds_method_stallguard,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0
    };
    
    orient_item_ = {
        .label = "Orientation",
        .type = MenuItemType::TOGGLE,
        .data = &settings_->orientation_flipped,
        .parent = &settings_menu_,
        .children = nullptr,
        .child_count = 0
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
                display_->fillRect(0, y - 2, 128, 10, 1);
                display_->setTextColor(0);
            } else {
                display_->setTextColor(1);
            }
            
            display_->setCursor(4, y);
            display_->print(item->label);
            
            y += 12;
        }
    }
    
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
    display_->print("Edit Value");
    
    // Draw value (large)
    display_->setTextSize(2);
    display_->setCursor(0, 20);
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", *edit_value_ptr_);
    display_->print(buf);
    
    // Draw range
    display_->setTextSize(1);
    display_->setCursor(0, 45);
    snprintf(buf, sizeof(buf), "Range: %lu - %lu", edit_min_, edit_max_);
    display_->print(buf);
    
    // Draw help
    display_->setCursor(0, 55);
    display_->print("Rotate: Adjust  BACK: Cancel");
    
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

void OLEDMenu::enterItem() {
    if (!current_menu_->children || current_index_ >= static_cast<int>(current_menu_->child_count)) {
        return;
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
        } else if (strcmp(item->label, "Dwell") == 0) {
            startValueEdit(value_ptr, 0, 60, 1);
        }
    } else if (item->type == MenuItemType::TOGGLE) {
        bool* value_ptr = static_cast<bool*>(item->data);
        *value_ptr = !(*value_ptr);
    }
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

