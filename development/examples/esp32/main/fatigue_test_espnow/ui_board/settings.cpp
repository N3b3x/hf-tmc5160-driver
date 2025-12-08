/**
 * @file settings.cpp
 * @brief Settings storage implementation
 */

#include "settings.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_crc.h"

static const char* TAG_SET = "Settings";

namespace {

const char* NVS_NAMESPACE = "fatigue";
const char* KEY_BLOB      = "cfg_blob";
const char* KEY_CRC       = "cfg_crc";

} // namespace

// Helper to validate boolean values are strictly 0 or 1
static bool validate_bool(uint8_t val) {
    return (val == 0 || val == 1);
}

void SettingsStore::init(Settings& s)
{
    // 1. Initialize with defaults first
    s = Settings{}; 

    // 2. Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    nvs_handle_t h;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h); // Open as READWRITE to allow repair
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SET, "Failed to open NVS: %s", esp_err_to_name(err));
        return;
    }

    // 3. Attempt to read blob and CRC
    size_t required_size = sizeof(Settings);
    size_t blob_size = 0;
    uint32_t stored_crc = 0;
    Settings loaded_s;

    // Check if blob exists and has correct size
    err = nvs_get_blob(h, KEY_BLOB, NULL, &blob_size);
    
    bool load_success = false;

    if (err == ESP_OK && blob_size == required_size) {
        // Read the blob
        err = nvs_get_blob(h, KEY_BLOB, &loaded_s, &blob_size);
        if (err == ESP_OK) {
            // Read the stored CRC
            if (nvs_get_u32(h, KEY_CRC, &stored_crc) == ESP_OK) {
                // Calculate CRC of loaded data
                uint32_t calc_crc = esp_crc32_le(0, (const uint8_t*)&loaded_s, sizeof(Settings));
                
                if (calc_crc == stored_crc) {
                    // CRC matches, now validate logical constraints (bools)
                    // We cast to uint8_t* to inspect the raw bytes of the bool fields
                    uint8_t* p_orient = (uint8_t*)&loaded_s.ui.orientation_flipped;
                    uint8_t* p_bounds = (uint8_t*)&loaded_s.test_unit.bounds_method_stallguard;

                    if (validate_bool(*p_orient) && validate_bool(*p_bounds)) {
                        ESP_LOGI(TAG_SET, "Settings loaded and verified (CRC: 0x%08lx)", stored_crc);
                        s = loaded_s;
                        load_success = true;
                    } else {
                        ESP_LOGE(TAG_SET, "Settings CRC OK but bool validation failed! (orient=%u, bounds=%u)", *p_orient, *p_bounds);
                    }
                } else {
                    ESP_LOGW(TAG_SET, "Settings CRC mismatch! Stored: 0x%08lx, Calc: 0x%08lx", stored_crc, calc_crc);
                }
            } else {
                ESP_LOGW(TAG_SET, "Settings blob found but CRC missing");
            }
        }
    } else {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG_SET, "No settings found in NVS, creating defaults");
        } else {
            ESP_LOGW(TAG_SET, "Settings blob size mismatch or read error (sz=%d, exp=%d)", (int)blob_size, (int)required_size);
        }
    }

    // 4. If load failed (corruption, missing, size mismatch), save defaults
    if (!load_success) {
        ESP_LOGW(TAG_SET, "Using defaults and overwriting NVS");
        // s is already default from step 1
        
        // Calculate CRC for defaults
        uint32_t crc = esp_crc32_le(0, (const uint8_t*)&s, sizeof(Settings));
        
        // Save
        nvs_set_blob(h, KEY_BLOB, &s, sizeof(Settings));
        nvs_set_u32(h, KEY_CRC, crc);
        nvs_commit(h);
    }

    nvs_close(h);
}

void SettingsStore::save(const Settings& s)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SET, "Failed to open NVS for write: %s", esp_err_to_name(err));
        return;
    }

    // Calculate CRC
    uint32_t crc = esp_crc32_le(0, (const uint8_t*)&s, sizeof(Settings));

    // Write blob and CRC
    err = nvs_set_blob(h, KEY_BLOB, &s, sizeof(Settings));
    if (err == ESP_OK) {
        err = nvs_set_u32(h, KEY_CRC, crc);
    }

    if (err == ESP_OK) {
        err = nvs_commit(h);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_SET, "Settings saved (CRC: 0x%08lx)", crc);
        } else {
            ESP_LOGE(TAG_SET, "NVS commit failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG_SET, "NVS write failed: %s", esp_err_to_name(err));
    }

    nvs_close(h);
}
