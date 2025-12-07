/**
 * @file main.cpp
 * @brief Main application for OLED UI board (remote controller)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"

#include "config.hpp"
#include "espnow_protocol.hpp"
#include "button.hpp"
#include "settings.hpp"
#include "ui_oled.hpp"  // OLED-based UI (replaces e-ink ui.hpp)

static const char* TAG_MAIN = "Main";

static QueueHandle_t g_buttonQueue = nullptr;
static QueueHandle_t g_protoQueue  = nullptr;
static QueueHandle_t g_uiQueue     = nullptr;

// Used for inactivity->deep sleep
static uint32_t g_lastActivityTick = 0;

// Forward tasks
static void button_task(void* arg);
static void proto_task(void* arg);
static void power_task(void* arg);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "Boot, wakeup cause: %d", (int)esp_sleep_get_wakeup_cause());

    Settings settings;
    SettingsStore::init(settings);

    g_buttonQueue = xQueueCreate(10, sizeof(ButtonEvent));
    g_protoQueue  = xQueueCreate(10, sizeof(ProtoEvent));
    g_uiQueue     = xQueueCreate(10, sizeof(UiEvent));

    // Init ESPNOW (UI side)
    EspNowProto::init(g_protoQueue);

    // Buttons (ISR->g_buttonQueue)
    Buttons::init(g_buttonQueue);

    // Configure deep sleep wake from buttons
    Buttons::configure_wakeup();

    // Initialize OLED-based UI (replaces e-ink display)
    g_lastActivityTick = xTaskGetTickCount();
    UI_OLED::init(g_uiQueue, &settings, &g_lastActivityTick);

    // Request config from test unit at startup
    EspNowProto::send_config_request();

    // Launch tasks
    xTaskCreate(button_task, "button_task", 4096, nullptr, 6, nullptr);
    xTaskCreate(proto_task,  "proto_task",  4096, nullptr, 5, nullptr);
    xTaskCreate(UI_OLED::task, "ui_task",   8192, nullptr, 4, nullptr);  // Increased stack for OLED UI
    xTaskCreate(power_task,  "power_task",  4096, nullptr, 3, nullptr);
}

// Button task: forward button events into UI queue
static void button_task(void* arg)
{
    ButtonEvent be{};
    while (true) {
        if (xQueueReceive(g_buttonQueue, &be, portMAX_DELAY) == pdTRUE) {
            UiEvent ev{};
            ev.type = UiEventType::BTN;
            ev.data.btn = be;
            xQueueSend(g_uiQueue, &ev, 0);
        }
    }
}

// Proto task: forward protocol events into UI queue
static void proto_task(void* arg)
{
    ProtoEvent pe{};
    while (true) {
        if (xQueueReceive(g_protoQueue, &pe, portMAX_DELAY) == pdTRUE) {
            UiEvent ev{};
            ev.type = UiEventType::PROTO;
            ev.data.proto = pe;
            xQueueSend(g_uiQueue, &ev, 0);
        }
    }
}

// Power task: check inactivity and enter deep sleep
static void power_task(void* arg)
{
    const TickType_t checkPeriod = pdMS_TO_TICKS(1000);
    while (true) {
        vTaskDelay(checkPeriod);

        TickType_t now = xTaskGetTickCount();
        TickType_t timeoutTicks = pdMS_TO_TICKS(INACTIVITY_TIMEOUT_SEC * 1000);
        if (now - g_lastActivityTick > timeoutTicks) {
            ESP_LOGI(TAG_MAIN, "Inactivity timeout reached, entering deep sleep");

            vTaskDelay(pdMS_TO_TICKS(100));
            esp_deep_sleep_start();
        }
    }
}
