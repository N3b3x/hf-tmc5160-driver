# Hardware Setup Guide - ESP32 UI Slideshow

## Overview

This guide covers the hardware setup for the ESP32 UI Slideshow application, including display connections, SD card setup, and button wiring.

## Hardware Requirements

### Required Components

1. **ESP32 Development Board** (ESP32-C6 recommended)
2. **Adafruit 2.9" ThinkInk FeatherWing Tricolor E-Ink Display**
3. **MicroSD Card** (FAT32 formatted)
4. **SD Card Adapter/Module** (SPI interface)
5. **Three Buttons** (for navigation)
6. **Power Supply** (for ESP32)

## Pin Configuration

### E-Ink Display (SPI)

Edit `main/config.hpp` to match your wiring:

```cpp
// E-ink display pins
static constexpr int EINK_DC_PIN    = 12;   // Data/Command pin
static constexpr int EINK_RESET_PIN = 13;   // Reset pin
static constexpr int EINK_CS_PIN    = 14;   // Chip Select pin
static constexpr int EINK_BUSY_PIN  = 15;   // Busy pin (optional)

// SPI bus pins
static constexpr gpio_num_t SPI_SCK_PIN  = GPIO_NUM_18;  // Clock
static constexpr gpio_num_t SPI_MOSI_PIN = GPIO_NUM_23;  // MOSI
static constexpr gpio_num_t SPI_MISO_PIN = GPIO_NUM_19;  // MISO
```

### SD Card (SPI, Shared Bus)

```cpp
// SD card chip select (shares SPI bus with display)
static constexpr gpio_num_t SD_CS_PIN = GPIO_NUM_5;
```

**Note**: SD card shares SPI bus (SCK, MOSI, MISO) with display but uses separate CS pin.

### Buttons

```cpp
// Button GPIOs (must be RTC-capable for deep sleep wake)
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_9;   // Previous
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_10;  // Toggle auto-advance
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_11;  // Next
```

## Wiring Diagram

```
ESP32-C6                    E-Ink Display
┌─────────┐                 ┌──────────────┐
│ GPIO 18 │─────────────────│ SCK (Clock)  │
│ GPIO 23 │─────────────────│ MOSI (Data)  │
│ GPIO 19 │─────────────────│ MISO (N/A)   │
│ GPIO 14 │─────────────────│ CS           │
│ GPIO 12 │─────────────────│ DC           │
│ GPIO 13 │─────────────────│ RST          │
│ GPIO 15 │─────────────────│ BUSY         │
└─────────┘                 └──────────────┘

ESP32-C6                    SD Card Module
┌─────────┐                 ┌──────────────┐
│ GPIO 18 │─────────────────│ SCK          │
│ GPIO 23 │─────────────────│ MOSI         │
│ GPIO 19 │─────────────────│ MISO         │
│ GPIO 5  │─────────────────│ CS           │
└─────────┘                 └──────────────┘

ESP32-C6                    Buttons
┌─────────┐                 ┌──────────────┐
│ GPIO 9  │─────────────────│ UP Button    │
│ GPIO 10 │─────────────────│ SELECT Button│
│ GPIO 11 │─────────────────│ DOWN Button  │
│ 3.3V    │─────────────────│ (Pull-up)    │
└─────────┘                 └──────────────┘
```

## Display Setup

### ThinkInk FeatherWing

The 2.9" ThinkInk FeatherWing is designed for Feather-compatible boards:

- **Controller**: IL0373
- **Resolution**: 296x128 (horizontal) / 128x296 (portrait)
- **Colors**: Black, White, Red (tricolor)
- **Interface**: SPI
- **Power**: 3.3V

### Display Initialization

The display is initialized in `slideshow.cpp`:

```cpp
g_display = new Adafruit_IL0373(
    EINK_CS_PIN,
    EINK_DC_PIN,
    EINK_RESET_PIN,
    EINK_BUSY_PIN,
    296,   // width (native horizontal)
    128    // height (native horizontal)
);
```

**Note**: Display is used in portrait mode (128x296) after rotation.

## SD Card Setup

### Card Requirements

- **Format**: FAT32
- **Capacity**: Any size (tested up to 32GB)
- **Speed Class**: Class 10 recommended
- **File System**: Standard FAT32

### Preparing SD Card

1. **Format Card**:
   ```bash
   # Linux/macOS
   sudo mkfs.vfat -F 32 /dev/sdX1
   
   # Windows
   # Use Disk Management or format tool
   ```

2. **Create Directory**:
   ```bash
   mkdir /path/to/sdcard/images
   ```

3. **Copy Images**:
   ```bash
   cp *.bmp /path/to/sdcard/images/
   ```

### SD Card Mount Point

- **Mount Point**: `/sdcard`
- **Image Directory**: `/sdcard/images`
- **Auto-mount**: Handled by application

## Button Setup

### Button Type

- **Type**: Momentary push buttons
- **Configuration**: Active-low with pull-up resistors
- **Debounce**: 50ms (configurable)

### Wiring

- **Common**: Connect all button commons to GND
- **Signal**: Connect each button signal to respective GPIO
- **Pull-up**: Internal pull-ups enabled (or external 10kΩ)

### Deep Sleep Wake

Buttons must be RTC-capable GPIOs for deep sleep wake:
- Check ESP32-C6 RTC GPIO list
- Configure wake sources in `button.cpp`

## Power Supply

### Requirements

- **Voltage**: 3.3V or 5V (depending on ESP32 board)
- **Current**: ~200-500mA typical
- **Regulation**: Stable voltage

### Power Considerations

- **Display Refresh**: Higher current during refresh (~100-200mA)
- **SD Card Access**: Moderate current during file access
- **Deep Sleep**: Very low current (< 10μA)

## Initial Testing

### 1. Serial Connection

Connect to ESP32 via USB:

```bash
# Find serial port
ls /dev/ttyUSB*  # Linux
ls /dev/tty.*    # macOS

# Connect
screen /dev/ttyUSB0 115200
```

### 2. Verify Initialization

Look for startup messages:
```
[Slideshow] Initializing slideshow...
[SDCard] SD card mounted successfully
[Slideshow] Found 5 images
```

### 3. Test Display

- Display should show first image
- Check image quality and colors
- Verify display refresh works

### 4. Test Buttons

- **UP**: Should show previous image
- **SELECT**: Should toggle auto-advance
- **DOWN**: Should show next image

### 5. Test SD Card

- Verify images are detected
- Check file listing in serial output
- Test image loading

## Troubleshooting

### Display Not Working

1. **Check SPI Wiring**:
   - Verify SCK, MOSI, CS, DC, RST connections
   - Check for loose connections
   - Verify pin numbers in config

2. **Check Power**:
   - Verify 3.3V power to display
   - Check current capacity
   - Verify power sequencing

3. **Check Initialization**:
   - Look for display init messages
   - Check for error messages
   - Verify display object creation

### SD Card Not Detected

1. **Check Wiring**:
   - Verify SPI connections (SCK, MOSI, MISO, CS)
   - Check CS pin is correct
   - Verify card is inserted properly

2. **Check Format**:
   - Ensure card is FAT32
   - Check for corruption
   - Try reformatting

3. **Check Mount**:
   - Look for mount messages
   - Check for mount errors
   - Verify mount point

### Images Not Displaying

1. **Check File Format**:
   - Must be Windows BMP
   - Must be uncompressed
   - Check file is not corrupted

2. **Check Location**:
   - Must be in `/sdcard/images/` directory
   - Check filename extension (.bmp or .BMP)
   - Verify file permissions

3. **Check Memory**:
   - Large images may cause memory issues
   - Check available RAM
   - Try smaller images

### Buttons Not Working

1. **Check Wiring**:
   - Verify GPIO connections
   - Check button type (active-low)
   - Verify pull-up resistors

2. **Check Configuration**:
   - Verify GPIO numbers in config
   - Check debounce settings
   - Verify interrupt configuration

3. **Check Deep Sleep**:
   - Buttons must be RTC-capable
   - Check wake configuration
   - Verify wake sources

## Calibration

### Display Calibration

- Display should work out-of-the-box
- No calibration needed for IL0373
- Check rotation if display is upside-down

### Button Calibration

- Buttons should work with default debounce (50ms)
- Adjust if buttons are too sensitive or not responsive
- Check for bounce in serial output

## Maintenance

1. **Regular Checks**:
   - SD card connections
   - Display connections
   - Button functionality

2. **Software Updates**:
   - Keep firmware updated
   - Check for configuration changes

3. **SD Card Maintenance**:
   - Periodically check for corruption
   - Backup images
   - Format if issues occur

## Safety Considerations

1. **Power Supply**: Use appropriate power supply for ESP32
2. **SD Card**: Handle SD card carefully (static sensitive)
3. **Display**: E-ink display is fragile, handle with care
4. **Wiring**: Ensure proper connections to avoid shorts

