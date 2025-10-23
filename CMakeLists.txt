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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

set(PICO_SDK_PATH ~/Users/kazp/Documents/pico-sdk)

if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(LuminiferousEther C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(LuminiferousEther
            LuminiferousEther.cpp
            ADC_SPI/MCP3x6x_ADC.cpp
            ADC_SPI/MCP3x6x_Peripheral.cpp
            ADC_SPI/MCP3x6x_SPI.cpp
            )

pico_set_program_name(LuminiferousEther "LuminiferousEther")
pico_set_program_version(LuminiferousEther "0.1")

# Generate PIO header
pico_generate_pio_header(LuminiferousEther ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(LuminiferousEther 0)
pico_enable_stdio_usb(LuminiferousEther 1)

# Add the standard library to the build
target_link_libraries(LuminiferousEther
        pico_stdlib
        hardware_interp
)

# Add the standard include files to the build
target_include_directories(LuminiferousEther PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(LuminiferousEther 
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_clocks
        )

pico_add_extra_outputs(LuminiferousEther)

