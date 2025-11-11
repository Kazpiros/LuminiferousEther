#ifndef LE_DEFS_H
#define LE_DEFS_H

#define SAMPLE_RATE 22e3
#define OUTPUT_RATE 1e5 // idk if this is even right
#define DECIMATION_FACTOR () (static_cast<int>(SAMPLING_FREQUENCY / OUTPUT_FREQUENCY))

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"

#include "ADC_SPI/MCP3x6x_Type_Definitions.h"                                                           // Variable definitions header-file.
#include "ADC_SPI/MCP3x6x_ADC_Definitions.h"                                                            // ADC definitions header-file.
#include "ADC_SPI/MCP3x6x_Peripheral_Definitions.h"                                                     // Peripheral definitions header-file.
#include "ADC_SPI/MCP3x6x_SPI_Definitions.h"

#include "include/NCO.hpp"
#include "include/cbuf.hpp"
#include "include/adc_spi_config.hpp"
#include "include/dac_spi_config.hpp"
#include "include/filter_coef.h"
#include "include/pio_spi.h"


#define SPI_PORT spi0 // all this will be removed soon
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_IRQ  20
#define PIN_MCLK 21

#define DAC0_WRITE_MASK 0x3FF

#endif