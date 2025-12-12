# E-Ink Slideshow Application Plan

## Overview
A standalone application that displays images from an SD card on the 2.9" ThinkInk e-ink display. Uses the legacy buttons (UP, SELECT, DOWN) for navigation.

## Hardware Requirements
- **Display**: Adafruit 2.9" ThinkInk FeatherWing Tricolor E-Ink Display (IL0373)
  - Resolution: 296x128 pixels (horizontal) / 128x296 pixels (portrait)
  - Colors: Black, White, Red (tricolor)
  - Interface: SPI
- **SD Card**: MicroSD card slot (SPI interface)
  - Format: FAT32
  - Supported formats: BMP images (1-bit, 4-bit, or 8-bit)
- **Buttons**: Three buttons for navigation
  - UP: Previous image
  - SELECT: Toggle auto-advance / Favorite image
  - DOWN: Next image

## Software Architecture

### Folder Structure
```
ui_slideshow_board/
├── main.cpp              # Application entry point
├── config.hpp            # Hardware configuration (pins, display settings)
├── slideshow.hpp/cpp     # Main slideshow logic and state machine
├── sd_card.hpp/cpp       # SD card initialization and file system access
├── image_loader.hpp/cpp  # Image loading and format conversion
├── button.hpp/cpp        # Button handling (simplified, no ESP-NOW)
└── README.md             # Usage instructions
```

### Features

#### Core Features
1. **SD Card Detection**
   - Auto-detect SD card on startup
   - Scan for image files (BMP format)
   - Build image list in memory
   - Support for subdirectories (optional)

2. **Image Display**
   - Load BMP images from SD card
   - Convert to e-ink compatible format (1-bit or tricolor)
   - Display on e-ink screen with proper refresh
   - Handle different image sizes (scale/crop to fit)

3. **Navigation**
   - **UP Button**: Previous image
   - **SELECT Button**: 
     - Short press: Toggle auto-advance mode
     - Long press: Mark as favorite (future feature)
   - **DOWN Button**: Next image
   - Wrap around at beginning/end of list

4. **Auto-Advance Mode**
   - Automatically advance to next image after configurable delay
   - Pause on button press
   - Resume after inactivity timeout

5. **Display Optimization**
   - Full refresh only when necessary
   - Partial refresh for minor updates (if supported)
   - Proper e-ink refresh timing
   - Power management (deep sleep after inactivity)

#### Advanced Features (Future)
- Image metadata display (filename, date)
- Favorite images collection
- Image rotation/flip
- Slideshow settings (delay, shuffle, repeat)
- Multiple image format support (PNG, JPEG with conversion)

### State Machine

```
IDLE
  ├─> SCANNING (SD card detection, file discovery)
  ├─> DISPLAYING (showing current image)
  │     ├─> AUTO_ADVANCE (auto-advancing)
  │     └─> MANUAL (waiting for button input)
  └─> ERROR (SD card error, no images found)
```

### File Format Support

#### Primary: BMP
- 1-bit monochrome BMP (direct display)
- 4-bit grayscale BMP (convert to 1-bit)
- 8-bit grayscale BMP (convert to 1-bit)
- 24-bit RGB BMP (convert to tricolor: black/white/red)

#### Image Processing
- Resize/crop to fit display (128x296 portrait)
- Dithering for grayscale conversion
- Color quantization for tricolor mode

### Configuration

#### Hardware Pins (config.hpp)
- E-ink display pins (from ui_board/config.hpp)
- SD card SPI pins (CS, MOSI, MISO, SCK)
- Button GPIO pins (UP, SELECT, DOWN)

#### Runtime Settings
- Auto-advance delay (default: 10 seconds)
- Inactivity timeout (default: 5 minutes)
- Image directory path (default: "/images")
- Display orientation (portrait/landscape)

### Error Handling
- SD card not detected → Show error screen
- No images found → Show message, allow retry
- Image load failure → Skip to next image, log error
- Display refresh failure → Retry with full refresh

### Power Management
- Deep sleep after inactivity timeout
- Wake on button press
- Low power mode during auto-advance (CPU can sleep between images)

## Implementation Phases

### Phase 1: Basic Structure
- [x] Create folder structure
- [ ] Create main.cpp with basic initialization
- [ ] Create config.hpp with pin definitions
- [ ] Create button.hpp/cpp (simplified version)
- [ ] Add to build system

### Phase 2: SD Card Support
- [ ] Initialize SD card SPI interface
- [ ] Mount FAT filesystem
- [ ] Scan directory for BMP files
- [ ] Build image file list
- [ ] Error handling for SD card issues

### Phase 3: Image Display
- [ ] BMP file parsing
- [ ] Image format conversion (1-bit, grayscale, RGB)
- [ ] Display buffer management
- [ ] E-ink refresh optimization
- [ ] Image scaling/cropping

### Phase 4: Navigation
- [ ] Button event handling
- [ ] Image navigation (next/previous)
- [ ] Auto-advance mode
- [ ] State machine implementation

### Phase 5: Polish
- [ ] Error screens
- [ ] Loading indicators
- [ ] Settings persistence (NVS)
- [ ] Documentation

## Build System Integration

### app_config.yml
```yaml
fatigue_test_espnow_slideshow:
  description: "E-ink slideshow application - displays images from SD card"
  source_file: "fatigue_test_espnow/ui_slideshow_board/main.cpp"
  category: "display"
  idf_versions: ["release/v5.5"]
  build_types: ["Debug", "Release"]
  ci_enabled: false
  featured: false
```

### CMakeLists.txt
- Add slideshow source files
- Include Adafruit_EPD and Adafruit_GFX components
- Add SD card driver dependencies (esp_driver_sdspi)
- Add FAT filesystem support (fatfs component)

## Dependencies
- **Adafruit_EPD**: E-ink display driver
- **Adafruit_GFX**: Graphics primitives
- **Adafruit_BusIO_ESPIDF**: SPI communication
- **esp_driver_sdspi**: SD card SPI driver
- **fatfs**: FAT filesystem support
- **driver**: ESP-IDF driver framework
- **freertos**: Task management

## Testing Checklist
- [ ] SD card detection works
- [ ] Image files are discovered correctly
- [ ] Images display correctly on e-ink
- [ ] Navigation buttons work
- [ ] Auto-advance mode works
- [ ] Error handling works (no SD, no images, bad files)
- [ ] Power management works (deep sleep/wake)
- [ ] Display refresh is optimized

