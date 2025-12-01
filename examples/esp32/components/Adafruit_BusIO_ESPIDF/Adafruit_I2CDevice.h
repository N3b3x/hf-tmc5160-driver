// Minimal Adafruit_I2CDevice.h stub for ESP-IDF
// I2C is not needed for ThinkInk displays (SPI-only)
// This is a minimal stub to satisfy includes in Adafruit_GFX

#ifndef Adafruit_I2CDevice_h
#define Adafruit_I2CDevice_h

#include <Arduino.h>

// Minimal stub class (not implemented, SPI-only implementation)
class Adafruit_I2CDevice {
public:
    Adafruit_I2CDevice(uint8_t addr, void* theWire = nullptr) : _addr(addr) {}
    uint8_t address(void) { return _addr; }
    bool begin(bool addr_detect = true) { return false; }
    void end(void) {}
    bool detected(void) { return false; }
    bool read(uint8_t *buffer, size_t len, bool stop = true) { return false; }
    bool write(const uint8_t *buffer, size_t len, bool stop = true,
               const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0) { return false; }
    bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                         uint8_t *read_buffer, size_t read_len,
                         bool stop = false) { return false; }
    bool setSpeed(uint32_t desiredclk) { return false; }
    size_t maxBufferSize() { return 0; }

private:
    uint8_t _addr;
};

#endif // Adafruit_I2CDevice_h

