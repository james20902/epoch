# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion RISCV_PICO_2_3_1_0)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pimoroni_pico_plus2_rp2350 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(epoch C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(epoch
        src/main.c
        src/ahrs/lsm6dsox_reg.c
)

pico_set_program_name(epoch "epoch")
pico_set_program_version(epoch "0.1")

# no_flash means the target is to run from RAM
pico_set_binary_type(epoch no_flash)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(epoch 0)
pico_enable_stdio_usb(epoch 0)

# Add the standard library to the build
target_link_libraries(epoch
        pico_stdlib
        pico_multicore
        hardware_pwm
        hardware_spi)

# Add the standard include files to the build
target_include_directories(epoch PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/include
        # ST's vendor drivers include their headers by bare name ("lis3mdl_reg.h")
        ${CMAKE_CURRENT_LIST_DIR}/src/include/ahrs
)

pico_add_extra_outputs(epoch)

