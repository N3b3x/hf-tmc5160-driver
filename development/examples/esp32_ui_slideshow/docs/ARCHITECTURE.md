# ESP32 UI Slideshow - Architecture Documentation

## Overview

The ESP32 UI Slideshow is a standalone application for displaying images from an SD card on a 2.9" ThinkInk Tricolor E-Ink display. It features automatic image scanning, navigation controls, and power management.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Slideshow Application                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐ │
│  │   Button     │    │   SD Card    │    │   Image      │ │
│  │   Handler    │───▶│   Manager    │───▶│   Loader     │ │
│  │              │    │              │    │              │ │
│  └──────────────┘    └──────┬───────┘    └──────┬───────┘ │
│         │                    │                    │          │
│         │                    │                    │          │
│         └────────────────────┴────────────────────┘          │
│                            │                                  │
│                   ┌────────▼────────┐                         │
│                   │   Slideshow     │                         │
│                   │   Controller    │                         │
│                   └────────┬────────┘                         │
│                            │                                  │
│                   ┌────────▼────────┐                         │
│                   │  E-Ink Display  │                         │
│                   │  (IL0373)       │                         │
│                   └─────────────────┘                         │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Component Overview

### 1. Slideshow Controller

**Files**: `slideshow.hpp/cpp`

- **Purpose**: Main state machine and coordination
- **States**: INIT, SCANNING, DISPLAYING, AUTO_ADVANCE, ERROR, SLEEPING
- **Features**:
  - State machine management
  - Image file scanning
  - Auto-advance timing
  - Inactivity timeout
  - Deep sleep management

**Key Functions**:
- `Slideshow::init()` - Initialize slideshow system
- `Slideshow::task()` - Main slideshow task
- `Slideshow::getState()` - Get current state

### 2. SD Card Manager

**Files**: `sd_card.hpp/cpp`

- **Purpose**: SD card initialization and file system access
- **Interface**: FAT32 via ESP-IDF VFS
- **Features**:
  - SD card detection
  - FAT32 filesystem mounting
  - Directory scanning
  - File listing

**Key Functions**:
- `SDCard::init()` - Initialize SD card
- `SDCard::scanImages()` - Scan for image files
- `SDCard::getImageList()` - Get list of image files

### 3. Image Loader

**Files**: `image_loader.hpp/cpp`

- **Purpose**: Load and convert BMP images for display
- **Supported Formats**: 1-bit, 4-bit, 8-bit, 24-bit RGB BMP
- **Features**:
  - BMP header parsing
  - Format conversion
  - Color quantization (for tricolor)
  - Scaling/cropping to fit display

**Key Functions**:
- `ImageLoader::loadAndDisplayBMP()` - Load and display image
- `ImageLoader::rgbToEinkColor()` - Convert RGB to e-ink color

### 4. Button Handler

**Files**: `button.hpp/cpp`

- **Purpose**: Handle button input with debouncing
- **Buttons**: UP, SELECT, DOWN
- **Features**:
  - GPIO interrupt handling
  - Debouncing
  - Deep sleep wake support
  - Queue-based event delivery

**Key Functions**:
- `Buttons::init()` - Initialize buttons
- `Buttons::ConfigureWakeup()` - Configure deep sleep wake

### 5. E-Ink Display Driver

**Component**: `Adafruit_EPD` (IL0373 driver)

- **Purpose**: Control e-ink display
- **Interface**: SPI
- **Features**:
  - Display buffer management
  - Partial update support
  - Tricolor support (black/white/red)
  - Power management

## State Machine

```
┌─────────┐
│  INIT   │
└────┬────┘
     │
     ▼
┌──────────┐
│ SCANNING │───▶ (if no images) ──▶ ERROR
└────┬─────┘
     │
     ▼
┌────────────┐
│ DISPLAYING │◀──┐
└────┬───────┘   │
     │           │
     │ (button)  │ (auto-advance)
     ▼           │
┌──────────────┐ │
│ AUTO_ADVANCE │─┘
└────┬─────────┘
     │
     │ (timeout)
     ▼
┌──────────┐
│ SLEEPING │
└──────────┘
```

### State Descriptions

- **INIT**: Initializing hardware and scanning for images
- **SCANNING**: Scanning SD card for image files
- **DISPLAYING**: Displaying current image, waiting for input
- **AUTO_ADVANCE**: Automatically advancing through images
- **ERROR**: Error state (no images, SD card error, etc.)
- **SLEEPING**: Deep sleep mode (inactivity timeout)

## Image Format Support

### BMP Format Details

The loader supports standard Windows BMP format:

- **1-bit BMP**: Direct display (monochrome)
- **4-bit BMP**: Converted to 1-bit (grayscale)
- **8-bit BMP**: Converted to 1-bit (grayscale)
- **24-bit RGB BMP**: Converted to tricolor (black/white/red)

### Color Conversion

**Tricolor Mode** (24-bit RGB):
- Red channel > threshold → EPD_RED
- Brightness > threshold → EPD_WHITE
- Otherwise → EPD_BLACK

**Grayscale Mode** (4-bit, 8-bit):
- Brightness > threshold → EPD_WHITE
- Otherwise → EPD_BLACK

### Image Processing

1. **Header Parsing**: Read BMP header to determine format
2. **Format Detection**: Identify bits per pixel
3. **Data Reading**: Read pixel data from file
4. **Conversion**: Convert to e-ink format
5. **Scaling**: Scale/crop to fit display (128x296)
6. **Display**: Send to display buffer

## File System Structure

### SD Card Layout

```
/sdcard/
└── images/
    ├── image1.bmp
    ├── image2.bmp
    ├── image3.bmp
    └── ...
```

### Image Naming

- **Format**: Any valid filename
- **Extension**: `.bmp` or `.BMP`
- **Sorting**: Alphabetical by filename

## Power Management

### Deep Sleep

- **Trigger**: Inactivity timeout (default: 5 minutes)
- **Wake Sources**: Any button press
- **State**: All peripherals powered down
- **Recovery**: Full restart on wake

### Low Power Modes

- **Between Images**: CPU can sleep in auto-advance mode
- **Display Refresh**: E-ink display consumes power only during refresh

## Task Architecture

### Main Task

- **Name**: `slideshow_task`
- **Priority**: Normal
- **Stack Size**: 8192 bytes
- **Purpose**: Main slideshow state machine

### Button Task

- **Name**: `button_task`
- **Priority**: High (interrupt-driven)
- **Stack Size**: 2048 bytes
- **Purpose**: Process button events

## Memory Management

### Image Caching

- **File List**: Cached in memory (vector of strings)
- **Display Buffer**: Managed by Adafruit_EPD
- **BMP Buffer**: Temporary buffer for image loading

### Memory Constraints

- **ESP32-C6**: Limited RAM (~512KB)
- **Image Size**: Large images may cause memory issues
- **Recommendation**: Keep images reasonably sized

## Performance Considerations

### Display Refresh

- **Full Refresh**: ~2-3 seconds (e-ink limitation)
- **Partial Refresh**: Faster, but may cause ghosting
- **Optimization**: Minimize refresh frequency

### SD Card Access

- **SPI Speed**: Configured for SD card
- **File Reading**: Sequential access optimized
- **Directory Scanning**: Done once at startup

## Extension Points

1. **New Image Formats**: Extend `ImageLoader` class
2. **New Display Types**: Implement new display driver
3. **New Controls**: Add to button handler
4. **New Features**: Extend state machine

