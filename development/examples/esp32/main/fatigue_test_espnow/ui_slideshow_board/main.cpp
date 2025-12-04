/**
 * @file main.cpp
 * @brief E-Ink Slideshow Application
 * 
 * Displays images from SD card on 2.9" ThinkInk e-ink display.
 * Uses buttons for navigation (UP/DOWN/SELECT).
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"

#include "config.hpp"
#include "slideshow.hpp"
#include "button.hpp"

static const char* TAG_MAIN = "SlideshowMain";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "E-Ink Slideshow Application Starting...");
    ESP_LOGI(TAG_MAIN, "Wakeup cause: %d", (int)esp_sleep_get_wakeup_cause());

    // Initialize slideshow system
    if (!Slideshow::init()) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize slideshow");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }

    // Configure deep sleep wake from buttons
    SlideshowButtons::configure_wakeup();

    // Launch slideshow task
    xTaskCreate(Slideshow::task, "slideshow_task", 8192, nullptr, 5, nullptr);

    ESP_LOGI(TAG_MAIN, "Slideshow application started");
}

