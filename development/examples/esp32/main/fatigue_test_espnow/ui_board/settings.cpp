/**
 * @file settings.cpp
 * @brief Settings storage implementation
 */

#include "settings.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char* TAG_SET = "Settings";

namespace {

const char* NVS_NAMESPACE = "fatigue";
const char* KEY_CYCLES    = "cycles";
const char* KEY_TPER      = "tper";
const char* KEY_DWELL     = "dwell";
const char* KEY_ORIENT    = "orient";
const char* KEY_BOUNDS    = "bounds";

} // namespace

void SettingsStore::init(Settings& s)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    nvs_handle_t h;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_SET, "No existing settings, using defaults");
        return;
    }

    uint32_t val;
    if (nvs_get_u32(h, KEY_CYCLES, &val) == ESP_OK) s.cycle_amount = val;
    if (nvs_get_u32(h, KEY_TPER,   &val) == ESP_OK) s.time_per_cycle = val;
    if (nvs_get_u32(h, KEY_DWELL,  &val) == ESP_OK) s.dwell_time = val;
    uint8_t ov;
    if (nvs_get_u8(h, KEY_ORIENT, &ov) == ESP_OK) s.orientation_flipped = (ov != 0);
    if (nvs_get_u8(h, KEY_BOUNDS, &ov) == ESP_OK) s.bounds_method_stallguard = (ov == 0);

    nvs_close(h);
    ESP_LOGI(TAG_SET, "Settings loaded from NVS");
}

void SettingsStore::save(const Settings& s)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SET, "Failed to open NVS for write: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_u32(h, KEY_CYCLES, s.cycle_amount);
    nvs_set_u32(h, KEY_TPER,   s.time_per_cycle);
    nvs_set_u32(h, KEY_DWELL,  s.dwell_time);
    nvs_set_u8 (h, KEY_ORIENT, s.orientation_flipped ? 1 : 0);
    nvs_set_u8 (h, KEY_BOUNDS, s.bounds_method_stallguard ? 0 : 1);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG_SET, "Settings saved");
}
