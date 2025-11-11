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
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(le C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(le 
src/LuminiferousEther.cpp
src/cbuf.cpp
src/dac_spi_config.cpp
src/adc_spi_config.cpp
src/handlers.cpp
src/pio_spi.cpp
src/decimate.cpp

ADC_SPI/MCP3x6x_ADC.cpp
ADC_SPI/MCP3x6x_Peripheral.cpp
ADC_SPI/MCP3x6x_SPI.cpp
)

pico_set_program_name(le "le")
pico_set_program_version(le "0.1")

# Generate PIO header
pico_generate_pio_header(le 
    ${CMAKE_CURRENT_LIST_DIR}/src/spi.pio
    )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(le 0)
pico_enable_stdio_usb(le 0)

# Add the standard library to the build
target_link_libraries(le
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_clocks
        hardware_uart
        pico_binary_info
)

# Add the standard include files to the build
target_include_directories(le PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/ADC_SPI
)

pico_add_extra_outputs(le)

