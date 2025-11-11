#ifndef ADC_SPI_CONFIG_HPP
#define ADC_SPI_CONFIG_HPP

#include "le_defs.h"
#include "pio_spi.h"

void adc_config_set();
void pio_adc_config_set(const pio_spi_inst_t *spi, uint cs_pin);
//uint8_t pio_adc_conv_start(const pio_spi_inst_t *spi, uint8_t MUX_CHNL);

#endif