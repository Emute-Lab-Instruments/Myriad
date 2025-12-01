cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (must be before project)
include(../pico_sdk_import.cmake)

project(test_sinetable_rp2040 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(test_sinetable_rp2040
    test_sinetable_rp2040.cpp
)

# Include directories
target_include_directories(test_sinetable_rp2040 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../arduino_libmyriad
    ${CMAKE_CURRENT_SOURCE_DIR}/../Myriad_A
)

# Link libraries
target_link_libraries(test_sinetable_rp2040
    pico_stdlib
    hardware_interp
    hardware_timer
)

# Enable USB output, disable UART
pico_enable_stdio_usb(test_sinetable_rp2040 1)
pico_enable_stdio_uart(test_sinetable_rp2040 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(test_sinetable_rp2040)

# Print build info
message(STATUS "Building RP2040 Hardware Test")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
