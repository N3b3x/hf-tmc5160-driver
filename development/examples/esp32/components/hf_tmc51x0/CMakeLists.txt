###############################################################################
# HF TMC51x0 Driver - ESP-IDF Component Wrapper
###############################################################################
# Note:
#   cmake_minimum_required() is intentionally omitted here — ESP-IDF's
#   project.cmake already sets it and adding another can cause policy conflicts.
###############################################################################

set(HF_TMC51X0_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." CACHE PATH
    "Path to root of hf-tmc5160-driver (auto-detected from component location)")

get_filename_component(HF_TMC51X0_ROOT "${HF_TMC51X0_ROOT}" ABSOLUTE)
if(NOT EXISTS "${HF_TMC51X0_ROOT}/cmake/hf_tmc51x0_build_settings.cmake")
    message(FATAL_ERROR
        "Cannot find TMC51x0 driver root.\n"
        "  Resolved path: ${HF_TMC51X0_ROOT}\n"
        "  Expected file: ${HF_TMC51X0_ROOT}/cmake/hf_tmc51x0_build_settings.cmake\n"
        "  Fix: set HF_TMC51X0_ROOT to the correct driver checkout path.")
endif()

include("${HF_TMC51X0_ROOT}/cmake/hf_tmc51x0_build_settings.cmake")

idf_component_register(
    SRCS ${HF_TMC51X0_SOURCE_FILES}
    INCLUDE_DIRS ${HF_TMC51X0_PUBLIC_INCLUDE_DIRS}
    REQUIRES ${HF_TMC51X0_IDF_REQUIRES}
)

if(HF_TMC51X0_SOURCE_FILES)
    target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
else()
    target_compile_features(${COMPONENT_LIB} INTERFACE cxx_std_20)
endif()

