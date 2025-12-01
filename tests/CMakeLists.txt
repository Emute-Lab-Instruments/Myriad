cmake_minimum_required(VERSION 3.13)
project(sinetable_tests CXX)

# C++20 standard (required for fixedpoint.hpp concepts)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/../arduino_libmyriad
    ${CMAKE_SOURCE_DIR}/../Myriad_A
)

# Test executables
add_executable(test_sinetable_fixed
    test_sinetable_fixed.cpp
)

add_executable(test_metaOscSinesFixed
    test_metaOscSinesFixed.cpp
)

# Link math library
target_link_libraries(test_sinetable_fixed m)
target_link_libraries(test_metaOscSinesFixed m)

# Enable testing
enable_testing()
add_test(NAME SinetableFixedTests COMMAND test_sinetable_fixed)
add_test(NAME MetaOscSinesFixedTests COMMAND test_metaOscSinesFixed)

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
