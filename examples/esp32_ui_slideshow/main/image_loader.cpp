/**
 * @file image_loader.cpp
 * @brief Image loading implementation
 */

#include "image_loader.hpp"
#include "sd_card.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "../components/Adafruit_EPD/src/Adafruit_EPD.h"
#include <cstring>
#include <algorithm>
#include <cmath>

static const char* TAG_IMG = "ImageLoader";

uint16_t ImageLoader::rgbToEinkColor(uint8_t r, uint8_t g, uint8_t b)
{
    // Simple color quantization for tricolor e-ink
    // Convert RGB to grayscale first
    uint8_t gray = (r * 30 + g * 59 + b * 11) / 100;
    
    // Determine if pixel should be red (warm colors)
    bool isRed = (r > 128 && r > g && r > b);
    
    // Determine if pixel should be black (dark) or white (light)
    if (gray < 85) {
        return EPD_BLACK;
    } else if (isRed && gray > 100) {
        return EPD_RED;
    } else {
        return EPD_WHITE;
    }
}

bool ImageLoader::loadAndDisplayBMP(const char* filepath, Adafruit_IL0373* display)
{
    if (!filepath || !display) {
        return false;
    }

    ESP_LOGI(TAG_IMG, "Loading image: %s", filepath);

    // Read file size
    int32_t fileSize = SDCard::getFileSize(filepath);
    if (fileSize < 0 || fileSize < static_cast<int32_t>(sizeof(BMPHeader))) {
        ESP_LOGE(TAG_IMG, "Invalid file size or cannot read file");
        return false;
    }

    // Allocate buffer for file
    uint8_t* fileBuffer = new uint8_t[fileSize];
    if (!fileBuffer) {
        ESP_LOGE(TAG_IMG, "Failed to allocate buffer");
        return false;
    }

    // Read entire file
    int32_t bytesRead = SDCard::readFile(filepath, fileBuffer, fileSize);
    if (bytesRead != fileSize) {
        ESP_LOGE(TAG_IMG, "Failed to read file completely");
        delete[] fileBuffer;
        return false;
    }

    // Parse BMP header
    BMPHeader* header = reinterpret_cast<BMPHeader*>(fileBuffer);
    
    // Check signature
    if (header->signature != 0x4D42) {  // "BM"
        ESP_LOGE(TAG_IMG, "Invalid BMP signature");
        delete[] fileBuffer;
        return false;
    }

    // Check bits per pixel (support 1, 4, 8, 24)
    if (header->bitsPerPixel != 1 && header->bitsPerPixel != 4 && 
        header->bitsPerPixel != 8 && header->bitsPerPixel != 24) {
        ESP_LOGE(TAG_IMG, "Unsupported bits per pixel: %d", header->bitsPerPixel);
        delete[] fileBuffer;
        return false;
    }

    // Get image dimensions
    uint32_t imgWidth = abs(header->width);
    uint32_t imgHeight = abs(header->height);
    bool topDown = (header->height < 0);  // Negative height means top-down

    ESP_LOGI(TAG_IMG, "BMP: %dx%d, %d bpp", imgWidth, imgHeight, header->bitsPerPixel);

    // Get pixel data pointer
    uint8_t* pixelData = fileBuffer + header->dataOffset;

    // Clear display
    display->clearBuffer();

    // Calculate scaling to fit display
    float scaleX = static_cast<float>(DISPLAY_WIDTH) / imgWidth;
    float scaleY = static_cast<float>(DISPLAY_HEIGHT) / imgHeight;
    float scale = std::min(scaleX, scaleY);  // Maintain aspect ratio

    uint32_t scaledWidth = static_cast<uint32_t>(imgWidth * scale);
    uint32_t scaledHeight = static_cast<uint32_t>(imgHeight * scale);
    uint32_t offsetX = (DISPLAY_WIDTH - scaledWidth) / 2;
    uint32_t offsetY = (DISPLAY_HEIGHT - scaledHeight) / 2;

    // Process pixels based on bit depth
    if (header->bitsPerPixel == 24) {
        // 24-bit RGB
        uint32_t rowSize = ((imgWidth * 3 + 3) / 4) * 4;  // Row size is padded to 4 bytes
        for (uint32_t y = 0; y < scaledHeight; y++) {
            uint32_t srcY = static_cast<uint32_t>((y / scale));
            if (!topDown) {
                srcY = imgHeight - 1 - srcY;  // BMP is typically bottom-up
            }
            
            for (uint32_t x = 0; x < scaledWidth; x++) {
                uint32_t srcX = static_cast<uint32_t>((x / scale));
                uint32_t pixelOffset = srcY * rowSize + srcX * 3;
                
                // BMP stores as BGR
                uint8_t b = pixelData[pixelOffset];
                uint8_t g = pixelData[pixelOffset + 1];
                uint8_t r = pixelData[pixelOffset + 2];
                
                uint16_t color = rgbToEinkColor(r, g, b);
                display->drawPixel(offsetX + x, offsetY + y, color);
            }
        }
    } else if (header->bitsPerPixel == 8) {
        // 8-bit grayscale (with palette)
        // Simplified: treat as grayscale
        uint32_t rowSize = ((imgWidth + 3) / 4) * 4;
        for (uint32_t y = 0; y < scaledHeight; y++) {
            uint32_t srcY = static_cast<uint32_t>((y / scale));
            if (!topDown) {
                srcY = imgHeight - 1 - srcY;
            }
            
            for (uint32_t x = 0; x < scaledWidth; x++) {
                uint32_t srcX = static_cast<uint32_t>((x / scale));
                uint8_t gray = pixelData[srcY * rowSize + srcX];
                
                uint16_t color = (gray < 128) ? EPD_BLACK : EPD_WHITE;
                display->drawPixel(offsetX + x, offsetY + y, color);
            }
        }
    } else if (header->bitsPerPixel == 1) {
        // 1-bit monochrome
        uint32_t rowSize = ((imgWidth + 7) / 8 + 3) / 4 * 4;
        for (uint32_t y = 0; y < scaledHeight; y++) {
            uint32_t srcY = static_cast<uint32_t>((y / scale));
            if (!topDown) {
                srcY = imgHeight - 1 - srcY;
            }
            
            for (uint32_t x = 0; x < scaledWidth; x++) {
                uint32_t srcX = static_cast<uint32_t>((x / scale));
                uint32_t byteOffset = srcY * rowSize + (srcX / 8);
                uint8_t bitOffset = 7 - (srcX % 8);
                uint8_t bit = (pixelData[byteOffset] >> bitOffset) & 1;
                
                uint16_t color = bit ? EPD_BLACK : EPD_WHITE;
                display->drawPixel(offsetX + x, offsetY + y, color);
            }
        }
    }

    // Refresh display
    display->display();
    
    delete[] fileBuffer;
    ESP_LOGI(TAG_IMG, "Image displayed successfully");
    return true;
}

