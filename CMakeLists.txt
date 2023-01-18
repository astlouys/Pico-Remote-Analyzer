cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(Pico-Remote-Analyzer C CXX)

# set (CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set (C_STANDARD 11)
set (CXX_STANDARD 17)
set (PICO_BOARD pico)

pico_sdk_init()

add_executable(Pico-Remote-Analyzer Pico-Remote-Analyzer.c)

# Send Pico's output to USB instead of UART (for debug purposes).
pico_enable_stdio_uart(Pico-Remote-Analyzer 1)
pico_enable_stdio_usb(Pico-Remote-Analyzer  1)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(Pico-Remote-Analyzer)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(Pico-Remote-Analyzer pico_stdlib hardware_adc pico_unique_id)