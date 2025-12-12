# ESP32 UI Slideshow - Standalone E-Ink Display Application

A standalone ESP32 application for displaying images from an SD card on a 2.9" ThinkInk Tricolor E-Ink display. This project is completely decoupled from the TMC driver library and can be used independently.

## Overview

This application displays BMP images from an SD card on an Adafruit 2.9" ThinkInk FeatherWing Tricolor E-Ink Display. It features:

- **E-Ink Display**: 2.9" ThinkInk FeatherWing (296x128 pixels horizontal, 128x296 portrait)
- **SD Card Support**: FAT32 filesystem, automatic image scanning
- **Image Formats**: BMP (1-bit, 4-bit, 8-bit, 24-bit RGB)
- **Navigation**: Three-button control (UP, SELECT, DOWN)
- **Auto-Advance**: Automatic slideshow mode with configurable delay
- **Power Management**: Deep sleep on inactivity, wake on button press

## Hardware Requirements

### Display
- **Model**: Adafruit 2.9" ThinkInk FeatherWing Tricolor E-Ink Display
- **Controller**: IL0373
- **Resolution**: 296x128 pixels (horizontal) / 128x296 pixels (portrait)
- **Colors**: Black, White, Red (tricolor)
- **Interface**: SPI

### SD Card
- **Format**: FAT32
- **Interface**: SPI (shares bus with display)
- **Supported Formats**: BMP images

### Buttons
- **UP**: Previous image
- **SELECT**: Toggle auto-advance / Favorite (future)
- **DOWN**: Next image

### GPIO Configuration

Edit `main/config.hpp` to configure GPIO pins:

```cpp
// E-ink display pins
static constexpr int EINK_DC_PIN    = 12;
static constexpr int EINK_RESET_PIN = 13;
static constexpr int EINK_CS_PIN    = 14;
static constexpr int EINK_BUSY_PIN  = 15;

// SPI bus pins
static constexpr gpio_num_t SPI_SCK_PIN  = GPIO_NUM_18;
static constexpr gpio_num_t SPI_MOSI_PIN = GPIO_NUM_23;
static constexpr gpio_num_t SPI_MISO_PIN = GPIO_NUM_19;

// SD card
static constexpr gpio_num_t SD_CS_PIN = GPIO_NUM_5;

// Buttons
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_9;
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_10;
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_11;
```

## Project Structure

```
esp32_ui_slideshow/
├── main/                    # Application source files
│   ├── main.cpp            # Application entry point
│   ├── config.hpp          # Hardware configuration
│   ├── slideshow.hpp/cpp   # Main slideshow logic
│   ├── sd_card.hpp/cpp     # SD card handling
│   ├── image_loader.hpp/cpp # Image loading and conversion
│   ├── button.hpp/cpp      # Button handling
│   └── CMakeLists.txt      # Main component build config
├── components/              # Display driver components
│   ├── Adafruit_EPD/       # E-ink display driver
│   ├── Adafruit_GFX/       # Graphics primitives
│   ├── Adafruit_BusIO_ESPIDF/ # SPI/I2C bus abstraction
│   └── Adafruit_SH1106_ESPIDF/ # OLED driver (for future use)
├── scripts/                 # Build and flash scripts
├── CMakeLists.txt          # Root project configuration
├── app_config.yml          # App configuration
└── README.md               # This file
```

## Building

### Prerequisites

- ESP-IDF v5.5 or v5.4
- Python 3.x
- CMake 3.16+

### Build Steps

1. **Set up ESP-IDF environment**:
   ```bash
   . $IDF_PATH/export.sh
   ```

2. **Build the application**:
   ```bash
   ./scripts/build_app.sh ui_slideshow Release
   ```

3. **Flash to device**:
   ```bash
   ./scripts/flash_app.sh ui_slideshow Release
   ```

## Usage

1. **Prepare SD Card**:
   - Format SD card as FAT32
   - Create `/images` directory on SD card
   - Copy BMP images to `/images` directory

2. **Power On**:
   - Insert SD card
   - Power on device
   - Application will scan for images and display first image

3. **Navigation**:
   - **UP Button**: Previous image
   - **SELECT Button**: Toggle auto-advance mode
   - **DOWN Button**: Next image

4. **Auto-Advance**:
   - Press SELECT to enable/disable auto-advance
   - Default delay: 10 seconds (configurable in `config.hpp`)

## Configuration

### Slideshow Settings

Edit `main/config.hpp`:

```cpp
// Auto-advance delay (seconds)
static constexpr uint32_t AUTO_ADVANCE_DELAY_SEC = 10;

// Inactivity timeout before deep sleep (seconds)
static constexpr uint32_t INACTIVITY_TIMEOUT_SEC = 300;  // 5 minutes

// Maximum number of images to cache
static constexpr size_t MAX_IMAGE_FILES = 100;

// Image directory on SD card
static constexpr const char* IMAGE_DIRECTORY = "/sdcard/images";
```

## Image Format Support

### Supported Formats
- **1-bit BMP**: Direct display (monochrome)
- **4-bit BMP**: Converted to 1-bit (grayscale)
- **8-bit BMP**: Converted to 1-bit (grayscale)
- **24-bit RGB BMP**: Converted to tricolor (black/white/red)

### Image Processing
- Automatic scaling/cropping to fit display (128x296 portrait)
- Color quantization for tricolor mode
- Dithering for grayscale conversion

## Power Management

- **Deep Sleep**: After inactivity timeout (default: 5 minutes)
- **Wake Sources**: Any button press
- **Low Power**: CPU can sleep between images in auto-advance mode

## Future Enhancements

- [ ] OLED display support (components already included)
- [ ] PNG/JPEG image support with conversion
- [ ] Image metadata display (filename, date)
- [ ] Favorite images collection
- [ ] Image rotation/flip
- [ ] Slideshow settings (delay, shuffle, repeat)
- [ ] Multiple image format support

## Dependencies

### ESP-IDF Components
- `driver` - GPIO, SPI drivers
- `esp_timer` - Timer functionality
- `freertos` - Task management
- `fatfs` - FAT filesystem
- `esp_driver_sdspi` - SD card SPI driver
- `vfs` - Virtual filesystem

### External Components
- **Adafruit_EPD**: E-ink display driver
- **Adafruit_GFX**: Graphics primitives
- **Adafruit_BusIO_ESPIDF**: SPI/I2C bus abstraction
- **Adafruit_SH1106_ESPIDF**: OLED driver (for future OLED support)

## No TMC Driver Dependencies

This project is **completely independent** of the TMC driver library. It can be:
- Used as a standalone project
- Moved to a separate repository
- Used as a reference for e-ink display applications

## Documentation

Comprehensive documentation is available:

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System architecture, component overview, and state machine
- **[IMAGE_FORMAT.md](IMAGE_FORMAT.md)** - Image format specifications and conversion details
- **[HARDWARE_SETUP.md](HARDWARE_SETUP.md)** - Hardware setup guide, pin configuration, and wiring diagrams

## Troubleshooting

### SD Card Not Detected
- Check SPI wiring (CS, MOSI, MISO, SCK)
- Verify SD card is formatted as FAT32
- Check SD card is inserted properly

### No Images Found
- Verify images are in `/images` directory on SD card
- Check image file format (must be BMP)
- Ensure images are not corrupted

### Display Not Updating
- Check SPI wiring for display
- Verify display power supply
- Check display busy pin connection

### Buttons Not Working
- Verify GPIO pin configuration in `config.hpp`
- Check button wiring (active-low with pull-ups)
- Ensure buttons are RTC-capable for deep sleep wake

## License

This project uses Adafruit libraries which have their own licenses. See component directories for license information.

## Contributing

This is a standalone project. For improvements or bug fixes, please create issues or pull requests in the appropriate repository.

