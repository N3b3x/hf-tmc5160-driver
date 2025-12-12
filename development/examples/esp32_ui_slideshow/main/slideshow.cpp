/**
 * @file slideshow.cpp
 * @brief Slideshow implementation
 */

#include "slideshow.hpp"
#include "config.hpp"
#include "sd_card.hpp"
#include "image_loader.hpp"
#include "button.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "../components/Adafruit_EPD/src/Adafruit_EPD.h"
#include "../components/Adafruit_BusIO_ESPIDF/SPI.h"
#include <cstring>
#include <algorithm>

static const char* TAG_SLIDE = "Slideshow";

// Display object
static Adafruit_IL0373* g_display = nullptr;

// State
static Slideshow::State s_state = Slideshow::State::INIT;
static std::vector<std::string> s_imageFiles;
static size_t s_currentImageIndex = 0;
static bool s_autoAdvance = false;
static TickType_t s_lastActivityTick = 0;
static TickType_t s_lastAutoAdvanceTick = 0;

// Queues
static QueueHandle_t s_buttonQueue = nullptr;

// Forward declarations
static void handleButton(SlideshowButtonEvent evt);
static void drawErrorScreen(const char* message);
static void drawLoadingScreen(const char* message);
static void displayCurrentImage();

bool Slideshow::init()
{
    ESP_LOGI(TAG_SLIDE, "Initializing slideshow...");

    // Create button queue
    s_buttonQueue = xQueueCreate(10, sizeof(SlideshowButtonEvent));
    if (!s_buttonQueue) {
        ESP_LOGE(TAG_SLIDE, "Failed to create button queue");
        return false;
    }

    // Initialize buttons
    if (!SlideshowButtons::init(s_buttonQueue)) {
        ESP_LOGE(TAG_SLIDE, "Failed to initialize buttons");
        return false;
    }

    // Initialize SPI bus
    SPI.begin(SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN);
    ESP_LOGI(TAG_SLIDE, "SPI bus initialized");

    // Initialize e-ink display
    g_display = new Adafruit_IL0373(
        EINK_CS_PIN,
        EINK_DC_PIN,
        EINK_RESET_PIN,
        EINK_BUSY_PIN,
        296,   // width (native horizontal)
        128    // height (native horizontal)
    );

    g_display->begin();
    g_display->setRotation(1);  // Portrait mode
    ESP_LOGI(TAG_SLIDE, "E-ink display initialized");

    // Show loading screen
    drawLoadingScreen("Initializing...");

    // Initialize SD card
    if (!SDCard::init()) {
        drawErrorScreen("SD card error");
        s_state = Slideshow::State::ERROR;
        return false;
    }

    // Scan for images
    drawLoadingScreen("Scanning images...");
    s_state = Slideshow::State::SCANNING;
    
    size_t imageCount = SDCard::scanForImages(IMAGE_DIRECTORY, s_imageFiles);
    if (imageCount == 0) {
        drawErrorScreen("No images found");
        s_state = Slideshow::State::ERROR;
        return false;
    }

    ESP_LOGI(TAG_SLIDE, "Found %zu images", imageCount);
    s_currentImageIndex = 0;
    s_state = Slideshow::State::DISPLAYING;
    s_lastActivityTick = xTaskGetTickCount();
    s_lastAutoAdvanceTick = xTaskGetTickCount();

    // Display first image
    displayCurrentImage();

    return true;
}

void Slideshow::task(void* arg)
{
    SlideshowButtonEvent btnEvt;

    while (true) {
        // Process button events
        if (xQueueReceive(s_buttonQueue, &btnEvt, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (btnEvt.pressed) {
                s_lastActivityTick = xTaskGetTickCount();
                handleButton(btnEvt);
            }
        }

        // Handle auto-advance
        if (s_state == Slideshow::State::DISPLAYING && s_autoAdvance) {
            TickType_t now = xTaskGetTickCount();
            TickType_t elapsed = now - s_lastAutoAdvanceTick;
            
            if (elapsed >= pdMS_TO_TICKS(AUTO_ADVANCE_DELAY_SEC * 1000)) {
                // Advance to next image
                s_currentImageIndex = (s_currentImageIndex + 1) % s_imageFiles.size();
                displayCurrentImage();
                s_lastAutoAdvanceTick = now;
            }
        }

        // Check inactivity timeout
        TickType_t now = xTaskGetTickCount();
        TickType_t inactivity = now - s_lastActivityTick;
        if (inactivity >= pdMS_TO_TICKS(INACTIVITY_TIMEOUT_SEC * 1000)) {
            ESP_LOGI(TAG_SLIDE, "Inactivity timeout, entering deep sleep");
            g_display->clearBuffer();
            g_display->setCursor(20, 140);
            g_display->print("Sleeping...");
            g_display->display();
            
            SlideshowButtons::configure_wakeup();
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_deep_sleep_start();
        }
    }
}

Slideshow::State Slideshow::getState()
{
    return s_state;
}

size_t Slideshow::getCurrentImageIndex()
{
    return s_currentImageIndex;
}

size_t Slideshow::getImageCount()
{
    return s_imageFiles.size();
}

static void handleButton(SlideshowButtonEvent evt)
{
    if (s_state != Slideshow::State::DISPLAYING) {
        return;
    }

    switch (evt.id) {
        case SlideshowButtonId::UP:
            // Previous image
            if (s_imageFiles.size() > 0) {
                s_currentImageIndex = (s_currentImageIndex == 0) ? 
                    s_imageFiles.size() - 1 : s_currentImageIndex - 1;
                displayCurrentImage();
                s_lastAutoAdvanceTick = xTaskGetTickCount();
            }
            break;

        case SlideshowButtonId::DOWN:
            // Next image
            if (s_imageFiles.size() > 0) {
                s_currentImageIndex = (s_currentImageIndex + 1) % s_imageFiles.size();
                displayCurrentImage();
                s_lastAutoAdvanceTick = xTaskGetTickCount();
            }
            break;

        case SlideshowButtonId::SELECT:
            // Toggle auto-advance
            s_autoAdvance = !s_autoAdvance;
            s_lastAutoAdvanceTick = xTaskGetTickCount();
            ESP_LOGI(TAG_SLIDE, "Auto-advance: %s", s_autoAdvance ? "ON" : "OFF");
            
            // Show brief indicator
            g_display->setTextSize(2);
            g_display->setTextColor(EPD_BLACK);
            g_display->fillRect(0, 0, 128, 30, EPD_WHITE);
            g_display->setCursor(10, 10);
            g_display->print(s_autoAdvance ? "AUTO" : "MANUAL");
            g_display->display();
            vTaskDelay(pdMS_TO_TICKS(1000));
            displayCurrentImage();
            break;
    }
}

static void displayCurrentImage()
{
    if (s_imageFiles.empty() || s_currentImageIndex >= s_imageFiles.size()) {
        return;
    }

    const std::string& imagePath = s_imageFiles[s_currentImageIndex];
    ESP_LOGI(TAG_SLIDE, "Displaying image %zu/%zu: %s", 
             s_currentImageIndex + 1, s_imageFiles.size(), imagePath.c_str());

    if (!ImageLoader::loadAndDisplayBMP(imagePath.c_str(), g_display)) {
        ESP_LOGW(TAG_SLIDE, "Failed to load image, skipping");
        // Skip to next image
        s_currentImageIndex = (s_currentImageIndex + 1) % s_imageFiles.size();
        if (s_currentImageIndex != 0) {  // Avoid infinite loop
            displayCurrentImage();
        }
    }
}

static void drawErrorScreen(const char* message)
{
    if (!g_display) return;

    g_display->clearBuffer();
    g_display->setTextSize(2);
    g_display->setTextColor(EPD_BLACK);
    g_display->setCursor(10, 100);
    g_display->print("ERROR");
    g_display->setTextSize(1);
    g_display->setCursor(10, 140);
    g_display->print(message);
    g_display->display();
}

static void drawLoadingScreen(const char* message)
{
    if (!g_display) return;

    g_display->clearBuffer();
    g_display->setTextSize(2);
    g_display->setTextColor(EPD_BLACK);
    g_display->setCursor(20, 120);
    g_display->print(message);
    g_display->display();
}

