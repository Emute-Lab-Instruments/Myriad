cmake_minimum_required(VERSION 3.28)
include(pico_sdk_import.cmake)
project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_VERBOSE_MAKEFILE ON)

# option(UNITB "MLP_XMOS")


# if (${MLP_XMOS})


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS_RELEASE "-Ofast -ffast-math -frename-registers")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

pico_sdk_init()

add_executable(myriad_A myriad_A.c)
pico_set_binary_type(myriad_A copy_to_ram)
pico_generate_pio_header(myriad_A ${CMAKE_CURRENT_LIST_DIR}/dsp_clock.pio )
pico_enable_stdio_usb(myriad_A 1)
pico_enable_stdio_uart(myriad_A 0)
pico_add_extra_outputs(myriad_A)
target_link_libraries(myriad_A pico_stdlib hardware_pio hardware_adc hardware_dma hardware_spi pico_multicore)


add_executable(myriad_B myriad_B.c)
pico_set_binary_type(myriad_B copy_to_ram)
pico_generate_pio_header(myriad_B ${CMAKE_CURRENT_LIST_DIR}/dsp_clock.pio )
pico_enable_stdio_usb(myriad_B 1)
pico_enable_stdio_uart(myriad_B 0)
pico_add_extra_outputs(myriad_B)
target_link_libraries(myriad_B pico_stdlib hardware_pio hardware_adc hardware_dma hardware_spi pico_multicore)
