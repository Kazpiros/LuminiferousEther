/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PIO_SPI_H
#define _PIO_SPI_H

#include <stdint.h>

#include "ADC_SPI/MCP3x6x_Type_Definitions.h"                                               // Variable definitions header-file.
#include "ADC_SPI/MCP3x6x_SPI_Definitions.h" 
#include "ADC_SPI/MCP3x6x_ADC_Definitions.h"                                                // ADC definitions header-file.
 

#include "hardware/pio.h"
#include "spi.pio.h"

//#include "pico/stdlib.h"

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_spi_inst_t;

void pio_spi_write8_blocking(const pio_spi_inst_t *spi, const uint8_t *src, size_t len);

void pio_spi_read8_blocking(const pio_spi_inst_t *spi, uint8_t *dst, size_t len);

void adc_spi_wr(const pio_spi_inst_t *spi, uint8_t REG_ADDR, uint32_t REG_CFG);

void pio_spi_write8_read8_blocking(const pio_spi_inst_t *spi, uint8_t *src, uint8_t *dst, size_t len);

#endif
