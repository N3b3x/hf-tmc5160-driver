# Adafruit EPD Library for ESP-IDF
# https://github.com/adafruit/Adafruit_EPD
# BSD License

cmake_minimum_required(VERSION 3.5)

# Collect all source files from src/ directory
file(GLOB_RECURSE EPD_SRCS 
    "src/*.cpp"
    "src/*.c"
)

# Collect all header files for include directory
set(EPD_INCLUDE_DIRS 
    "src"
    "."
)

idf_component_register(
    SRCS ${EPD_SRCS}
    INCLUDE_DIRS ${EPD_INCLUDE_DIRS} "../Adafruit_BusIO_ESPIDF"
    REQUIRES Adafruit_GFX Adafruit_BusIO_ESPIDF driver esp_timer freertos
)

