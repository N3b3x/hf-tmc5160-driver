/**
 * @file sd_card.hpp
 * @brief SD card detection and file system access
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>

namespace SDCard {

/**
 * @brief Initialize SD card and mount filesystem
 * @return true if successful, false otherwise
 */
bool init();

/**
 * @brief Deinitialize SD card and unmount filesystem
 */
void deinit();

/**
 * @brief Check if SD card is mounted
 * @return true if mounted, false otherwise
 */
bool isMounted();

/**
 * @brief Scan directory for image files
 * @param directory Directory path to scan (e.g., "/sdcard/images")
 * @param imageFiles Output vector of image file paths
 * @return Number of images found
 */
size_t scanForImages(const char* directory, std::vector<std::string>& imageFiles);

/**
 * @brief Read file from SD card
 * @param filepath Full path to file
 * @param buffer Output buffer
 * @param maxSize Maximum bytes to read
 * @return Number of bytes read, or -1 on error
 */
int32_t readFile(const char* filepath, uint8_t* buffer, size_t maxSize);

/**
 * @brief Get file size
 * @param filepath Full path to file
 * @return File size in bytes, or -1 on error
 */
int32_t getFileSize(const char* filepath);

} // namespace SDCard

