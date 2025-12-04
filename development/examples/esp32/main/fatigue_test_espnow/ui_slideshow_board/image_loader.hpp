/**
 * @file image_loader.hpp
 * @brief Image loading and format conversion for e-ink display
 */

#pragma once

#include <cstdint>
#include <cstddef>

// Forward declarations
class Adafruit_GFX;
class Adafruit_IL0373;

namespace ImageLoader {

/**
 * @brief BMP file header structure
 */
#pragma pack(push, 1)
struct BMPHeader {
    uint16_t signature;      // "BM" (0x4D42)
    uint32_t fileSize;
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t dataOffset;
    uint32_t headerSize;      // Usually 40 for BITMAPINFOHEADER
    int32_t  width;
    int32_t  height;
    uint16_t planes;          // Must be 1
    uint16_t bitsPerPixel;    // 1, 4, 8, 24
    uint32_t compression;     // 0 = none
    uint32_t imageSize;
    int32_t  xPixelsPerM;
    int32_t  yPixelsPerM;
    uint32_t colorsUsed;
    uint32_t colorsImportant;
};
#pragma pack(pop)

/**
 * @brief Load and display BMP image on e-ink display
 * @param filepath Path to BMP file
 * @param display Display object (Adafruit_IL0373)
 * @return true if successful, false otherwise
 */
bool loadAndDisplayBMP(const char* filepath, Adafruit_IL0373* display);

/**
 * @brief Convert RGB pixel to e-ink color
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return E-ink color (EPD_BLACK, EPD_WHITE, or EPD_RED)
 */
uint16_t rgbToEinkColor(uint8_t r, uint8_t g, uint8_t b);

} // namespace ImageLoader

