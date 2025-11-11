/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "include/pio_spi.h"
#include "ADC_SPI/MCP3x6x_Type_Definitions.h"

// Just 8 bit functions provided here. The PIO program supports any frame size
// 1...32, but the software to do the necessary FIFO shuffling is left as an
// exercise for the reader :)
//
// Likewise we only provide MSB-first here. To do LSB-first, you need to
// - Do shifts when reading from the FIFO, for general case n != 8, 16, 32
// - Do a narrow read at a one halfword or 3 byte offset for n == 16, 8
// in order to get the read data correctly justified. 

void __time_critical_func(pio_spi_write8_blocking)(const pio_spi_inst_t *spi, const uint8_t *src, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            (void) *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *dst, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_write8_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *src, uint8_t *dst,
                                                         size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(adc_spi_wr)(const pio_spi_inst_t *spi, uint8_t REG_ADDR, uint32_t REG_CFG)
{
    
    uint8_t temp[] = {0};
    _u32data_t WR_DATA;                                                             // REG_ADDR Register write-data variable.

    WR_DATA.DWORD = REG_CFG;                                                        // REG_ADDR Register write-data value. 
    //uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];
    

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _LOCK_) 
    {
        gpio_put(17, 0);                                             // Assert nCS Pin low.
        //SPI.transfer((REG_ADDR << 2) | _WRT_CTRL_);                                 // Write-CMD of REG_ADDR Register.                
        temp[0] = (REG_ADDR << 2) | _WRT_CTRL_;
        pio_spi_write8_blocking(spi, temp, count_of(temp));
        temp[0] = WR_DATA.BYTE.LOW;
        pio_spi_write8_blocking(spi, temp, count_of(temp));
        //SPI.transfer(WR_DATA.BYTE.LOW);                                             // 8-bit REG_ADDR Register write-data byte.
        
        gpio_put(17, 1);                                            // Assert nCS Pin high.
    } 
    if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_) 
    {
        gpio_put(17, 0);                                           // Assert nCS Pin low.
        
        //spi_transfer((REG_ADDR << 2) | _WRT_CTRL_); 
        temp[0] = (REG_ADDR << 2) | _WRT_CTRL_;                                // Write-CMD of REG_ADDR Register.                     
        pio_spi_write8_blocking(spi, temp, count_of(temp));
        //spi_transfer(WR_DATA.BYTE.UPPER);                                           // 24-bit REG_ADDR Register write-data upper-byte.
        temp[0] = WR_DATA.BYTE.UPPER;
        pio_spi_write8_blocking(spi, temp, count_of(temp));
        //spi_transfer(WR_DATA.BYTE.HIGH);                                            // 24-bit REG_ADDR Register write-data high-byte.
        temp[0] = WR_DATA.BYTE.HIGH;
        pio_spi_write8_blocking(spi, temp, count_of(temp));
        //spi_transfer(WR_DATA.BYTE.LOW);                                             // 24-bit REG_ADDR Register write-data low-byte. 
        temp[0] = WR_DATA.BYTE.LOW;
        pio_spi_write8_blocking(spi, temp, count_of(temp));

        gpio_put(17, 1);                                             // Assert nCS Pin high
    }  
} 