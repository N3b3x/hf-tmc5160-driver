/**
 * @file button.cpp
 * @brief Button handling implementation for slideshow
 */

#include "button.hpp"
#include "config.hpp"
#include "esp_sleep.h"
#include "esp_log.h"

static const char* TAG_BTN = "SlideshowButtons";

static QueueHandle_t s_btnQueue = nullptr;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    SlideshowButtonId realId = *(SlideshowButtonId*)arg;

    SlideshowButtonEvent ev{ realId, true };  // Press event
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_btnQueue, &ev, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

bool SlideshowButtons::init(QueueHandle_t evt_queue)
{
    s_btnQueue = evt_queue;

    gpio_config_t io_conf{};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // assuming buttons to GND, pull-ups
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    // Configure each button
    io_conf.pin_bit_mask = (1ULL << BTN_UP_GPIO) |
                           (1ULL << BTN_SELECT_GPIO) |
                           (1ULL << BTN_DOWN_GPIO);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    static SlideshowButtonId upId   = SlideshowButtonId::UP;
    static SlideshowButtonId selId  = SlideshowButtonId::SELECT;
    static SlideshowButtonId downId = SlideshowButtonId::DOWN;

    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_UP_GPIO, gpio_isr_handler, &upId));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_SELECT_GPIO, gpio_isr_handler, &selId));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_DOWN_GPIO, gpio_isr_handler, &downId));

    ESP_LOGI(TAG_BTN, "Slideshow buttons initialized");
    return true;
}

void SlideshowButtons::configure_wakeup()
{
    // All buttons as EXT1 wake sources (any low)
    uint64_t mask = (1ULL << BTN_UP_GPIO) |
                    (1ULL << BTN_SELECT_GPIO) |
                    (1ULL << BTN_DOWN_GPIO);

    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ALL_LOW);
    // NOTE: these GPIOs must be RTC-capable; adjust pins if necessary.
}

