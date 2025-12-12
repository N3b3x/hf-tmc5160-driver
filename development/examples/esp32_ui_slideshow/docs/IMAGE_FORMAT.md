# Image Format Specification

## Supported Formats

The slideshow application currently supports **Windows BMP** format only.

## BMP Format Support

### Supported Bit Depths

| Bits Per Pixel | Support | Conversion |
|----------------|---------|------------|
| 1-bit | ✅ Direct | Displayed as-is (monochrome) |
| 4-bit | ✅ Converted | Converted to 1-bit (grayscale) |
| 8-bit | ✅ Converted | Converted to 1-bit (grayscale) |
| 24-bit RGB | ✅ Converted | Converted to tricolor (black/white/red) |
| 32-bit RGBA | ❌ Not supported | - |

### BMP Header Structure

The loader reads the standard Windows BMP header:

```cpp
struct BMPHeader {
    uint16_t signature;      // "BM" (0x4D42)
    uint32_t fileSize;
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t dataOffset;     // Offset to pixel data
    uint32_t headerSize;     // Usually 40 (BITMAPINFOHEADER)
    int32_t  width;          // Image width (pixels)
    int32_t  height;         // Image height (pixels, can be negative)
    uint16_t planes;         // Must be 1
    uint16_t bitsPerPixel;   // 1, 4, 8, 24
    uint32_t compression;    // 0 = none (uncompressed)
    uint32_t imageSize;      // Size of pixel data
    // ... additional fields
};
```

## Display Specifications

### E-Ink Display

- **Model**: Adafruit 2.9" ThinkInk FeatherWing Tricolor
- **Resolution**: 128x296 pixels (portrait mode)
- **Colors**: Black, White, Red (tricolor)
- **Controller**: IL0373

### Display Modes

1. **Monochrome Mode**: Black and white only
2. **Tricolor Mode**: Black, white, and red

## Image Processing Pipeline

### 1. Header Reading

```
Read BMP Header
    │
    ├─▶ Verify signature ("BM")
    ├─▶ Read dimensions (width, height)
    ├─▶ Read bits per pixel
    └─▶ Read compression type
```

### 2. Format Detection

```
Check bitsPerPixel
    │
    ├─▶ 1-bit  → Monochrome (direct)
    ├─▶ 4-bit  → Grayscale (convert)
    ├─▶ 8-bit  → Grayscale (convert)
    └─▶ 24-bit → RGB (convert to tricolor)
```

### 3. Data Reading

- **1-bit**: Read bit-packed data
- **4-bit**: Read nibble-packed data with palette
- **8-bit**: Read byte data with palette
- **24-bit**: Read RGB triplets (BGR order in BMP)

### 4. Color Conversion

#### 1-bit BMP
- Direct mapping: 0 = black, 1 = white
- No conversion needed

#### 4-bit / 8-bit BMP
- Read palette (if present)
- Convert to grayscale: `gray = 0.299*R + 0.587*G + 0.114*B`
- Threshold: `gray > 128 ? white : black`

#### 24-bit RGB BMP
- Read RGB values (BGR order)
- Tricolor conversion:
  ```
  if (R > RED_THRESHOLD && R > G && R > B):
      color = EPD_RED
  else if (brightness > WHITE_THRESHOLD):
      color = EPD_WHITE
  else:
      color = EPD_BLACK
  ```

### 5. Scaling/Cropping

Images are scaled/cropped to fit display (128x296):

- **Aspect Ratio Preserved**: Center crop if needed
- **Scaling**: Nearest-neighbor interpolation
- **Orientation**: Portrait mode (128x296)

## Color Thresholds

### Tricolor Conversion Thresholds

```cpp
// Red detection
static constexpr uint8_t RED_THRESHOLD = 128;
static constexpr uint8_t RED_DOMINANCE = 1.2; // R must be 20% higher than G/B

// White detection
static constexpr uint8_t WHITE_THRESHOLD = 200;
static constexpr uint8_t MIN_BRIGHTNESS = 180; // Minimum for white
```

### Grayscale Conversion

```cpp
// Brightness calculation
uint8_t brightness = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);

// Threshold
static constexpr uint8_t GRAY_THRESHOLD = 128;
```

## Image Requirements

### Recommended Specifications

- **Format**: Windows BMP (uncompressed)
- **Bit Depth**: 24-bit RGB (best quality) or 1-bit (smallest size)
- **Dimensions**: 128x296 pixels (portrait) for best quality
- **File Size**: < 100KB recommended (memory constraints)

### Maximum Specifications

- **Width**: Any (will be scaled)
- **Height**: Any (will be scaled)
- **File Size**: Limited by available RAM (~200KB practical limit)

## Creating Compatible Images

### Using Image Editing Software

1. **Open Image**: In GIMP, Photoshop, or similar
2. **Resize**: Set to 128x296 pixels (portrait)
3. **Convert to Indexed**: For 1-bit or 4-bit
4. **Save as BMP**: Choose Windows BMP format
5. **Uncompressed**: Ensure no compression is used

### Using Command Line (ImageMagick)

```bash
# Convert to 1-bit BMP
convert input.jpg -resize 128x296 -monochrome output.bmp

# Convert to 24-bit RGB BMP
convert input.jpg -resize 128x296 -type TrueColor output.bmp
```

### Using Python (PIL/Pillow)

```python
from PIL import Image

# Load and convert
img = Image.open('input.jpg')
img = img.resize((128, 296), Image.LANCZOS)
img = img.convert('RGB')  # For 24-bit RGB
img.save('output.bmp', 'BMP')
```

## File Naming

- **Extension**: Must be `.bmp` or `.BMP`
- **Filename**: Any valid filename
- **Location**: Must be in `/sdcard/images/` directory
- **Sorting**: Alphabetical by filename

## Troubleshooting

### Image Not Displaying

1. **Check Format**: Must be Windows BMP
2. **Check Compression**: Must be uncompressed (compression = 0)
3. **Check File Size**: Not too large for available memory
4. **Check Location**: Must be in `/sdcard/images/` directory

### Colors Not Correct

1. **24-bit RGB**: Ensure image is 24-bit RGB (not indexed)
2. **Red Detection**: Red must be dominant color
3. **Brightness**: White requires high brightness

### Image Quality Issues

1. **Resolution**: Use 128x296 for best quality
2. **Scaling**: Nearest-neighbor may cause artifacts
3. **Dithering**: Consider dithering for grayscale conversion

## Future Enhancements

- PNG support with conversion
- JPEG support with conversion
- Image rotation/flip
- Better scaling algorithms (bilinear, bicubic)
- Dithering for better grayscale conversion

