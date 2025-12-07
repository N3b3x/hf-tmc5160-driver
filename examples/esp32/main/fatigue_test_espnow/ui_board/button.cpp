/**
 * @file button.cpp
 * @brief Button handling implementation
 */

#include "button.hpp"
#include "config.hpp"
#include "esp_sleep.h"
#include "esp_log.h"

static const char* TAG_BTN = "Buttons";

static QueueHandle_t s_btnQueue = nullptr;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    ButtonId realId = *(ButtonId*)arg;

    ButtonEvent ev{ realId };
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_btnQueue, &ev, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

bool Buttons::init(QueueHandle_t evt_queue)
{
    s_btnQueue = evt_queue;

    gpio_config_t io_conf{};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // assuming buttons to GND, pull-ups
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    // Configure OLED UI board buttons (BACK and CONFIRM only)
    io_conf.pin_bit_mask = (1ULL << BTN_BACK_GPIO) |
                           (1ULL << BTN_CONFIRM_GPIO);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // OLED UI board buttons
    static ButtonId backId = ButtonId::BACK;
    static ButtonId confirmId = ButtonId::CONFIRM;

    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_BACK_GPIO, gpio_isr_handler, &backId));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_CONFIRM_GPIO, gpio_isr_handler, &confirmId));

    ESP_LOGI(TAG_BTN, "Buttons initialized");
    return true;
}

void Buttons::configure_wakeup()
{
    // BACK and CONFIRM buttons as EXT1 wake sources (any low)
    uint64_t mask = (1ULL << BTN_BACK_GPIO) |
                    (1ULL << BTN_CONFIRM_GPIO);

    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ALL_LOW);
    // NOTE: these GPIOs must be RTC-capable; GPIO4 and GPIO6 are RTC-capable on ESP32C6
}
