###############################################################################
# TMC5160 Driver Component - CMakeLists.txt
###############################################################################
# This CMakeLists.txt configures the TMC5160 driver as an ESP-IDF component.
# The driver is header-only (template-based), so no source files need to be
# compiled. We just need to set up the include directories.
# 
# Note: src/ is included because tmc5160.cpp is in src/ and is included
# by tmc5160.hpp using a relative path.
###############################################################################

# Set the source root path relative to this component directory
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../")

idf_component_register(
    INCLUDE_DIRS "${SRC_ROOT}/inc" "${SRC_ROOT}/inc/features" "${SRC_ROOT}/inc/registers" "${SRC_ROOT}/src"
    REQUIRES driver
)

