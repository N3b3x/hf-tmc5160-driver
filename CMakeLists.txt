#===============================================================================
# TMC51x0 Driver - Root CMake
# Defines the main build target for the driver. Minimal, platform-agnostic.
#===============================================================================

cmake_minimum_required(VERSION 3.16)

project(hf_tmc51x0_driver LANGUAGES CXX)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/hf_tmc51x0_build_settings.cmake)

if(HF_TMC51X0_SOURCE_FILES)
    add_library(${HF_TMC51X0_TARGET_NAME} STATIC ${HF_TMC51X0_SOURCE_FILES})
    target_include_directories(${HF_TMC51X0_TARGET_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc/features>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc/registers>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
            $<BUILD_INTERFACE:${HF_TMC51X0_VERSION_HEADER_DIR}>
            $<INSTALL_INTERFACE:include>
    )
    target_compile_features(${HF_TMC51X0_TARGET_NAME} PUBLIC cxx_std_20)
else()
    add_library(${HF_TMC51X0_TARGET_NAME} INTERFACE)
    target_include_directories(${HF_TMC51X0_TARGET_NAME}
        INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc/features>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc/registers>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
            $<BUILD_INTERFACE:${HF_TMC51X0_VERSION_HEADER_DIR}>
            $<INSTALL_INTERFACE:include>
    )
    target_compile_features(${HF_TMC51X0_TARGET_NAME} INTERFACE cxx_std_20)
endif()

add_library(hf::tmc51x0 ALIAS ${HF_TMC51X0_TARGET_NAME})

set_target_properties(${HF_TMC51X0_TARGET_NAME} PROPERTIES VERSION ${HF_TMC51X0_VERSION} SOVERSION ${HF_TMC51X0_VERSION_MAJOR})

option(HF_TMC51X0_ENABLE_WARNINGS "Enable extra warnings for TMC51x0 driver" OFF)
if(HF_TMC51X0_ENABLE_WARNINGS)
    if(TARGET ${HF_TMC51X0_TARGET_NAME})
        target_compile_options(${HF_TMC51X0_TARGET_NAME} INTERFACE -Wall -Wextra -Wpedantic)
    endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${HF_TMC51X0_TARGET_NAME} EXPORT hf_tmc51x0Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY inc/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.h.in" EXCLUDE)
install(FILES "${HF_TMC51X0_VERSION_HEADER}" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT hf_tmc51x0Targets
    FILE hf_tmc51x0Targets.cmake
    NAMESPACE hf::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hf_tmc51x0
)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/hf_tmc51x0Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc51x0Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hf_tmc51x0
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc51x0ConfigVersion.cmake"
    VERSION ${HF_TMC51X0_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc51x0Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc51x0ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hf_tmc51x0
)
