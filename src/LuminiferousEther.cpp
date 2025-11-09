#include <cstdio>               // or <stdio.h>
//#include "pico/stdlib.h"
//#include <hardware/spi.h>
#include <cstdint>

//#include "kspi.pio.h"
#include "include/le_defs.h"
//#include "include/adc_spi_config.hpp"

//run with cmake --build build

/**
 * TODO:
 * - Need to add a #define for all CS pins so im not staring at "17"s
 * - Setup timer for output frequency control
 * - Make decimation filter to lower rate for DAC
 * - Delay effect + passthru + moving average 
 */

int irq_flag = 0;

uint8_t rxbuf[4];
uint8_t the[4] = {0x41,0,0,0};

int movingAve_coeff[10] = 
{
    21014991,
    72226277,
    182313887,
    337989153,
    460197516,
    460197516,
    337989153,
    182313887,
    72226277,
    21014991
};

void gpio_irq_handler(uint gpio, uint32_t events) { //The GPIO callback function to handle interrupts
    uint32_t sample = (rxbuf[0] << 24) | (rxbuf[1] << 16) | (rxbuf[2] << 8) | rxbuf[3];
    irq_flag = 1;
}

// spi is 1MHz, ~31ksps
// returns the most recent & processed sample from the ring. 
int moving_average(cbuf* inBuffer, cbuf* outBuffer)
{
    int NUM_TAPS = 10;
    int temp = inBuffer->read_cbuf(0);
    for(int i = 0; i < NUM_TAPS; i++)
    {
        temp += inBuffer->read_cbuf(i+1)*movingAve_coeff[i];
    }
    outBuffer->write_cbuf(temp);
    return outBuffer->read_cbuf(0);
}

void downsample()
{
    // need phase accumilator here
}

int main()
{
    
    stdio_init_all();
    cbuf InputBuffer(32);
    cbuf OutputBuffer(32);

    gpio_init(PIN_IRQ);
    gpio_set_dir(PIN_IRQ, GPIO_IN);
    gpio_pull_down(PIN_IRQ);
    // set up irq: call handler on both edges
    //gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_handler);


    spi_init(SPI_PORT, 1000*1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    //we use spi mode 1,1
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    
    
    adc_config_set();
    sleep_us(10);
    CONV_START(MUX_VINP_CH0);
    sleep_us(10);

    int temp = 0;
    NCO osc{};
    osc.frequency((float)OUTPUT_FREQUENCY);

    dac_config_set();

    while (1) {
        tight_loop_contents();
        gpio_put(PIN_CS, 0);
        
        temp = spi_write_read_blocking(SPI_PORT, the, rxbuf, 4); // clock 4 bytes, reading DATA0..DATA3
        gpio_put(PIN_CS, 1);
        
        uint32_t sample = (rxbuf[0] << 24) | (rxbuf[1] << 16) | (rxbuf[2] << 8) | rxbuf[3];
        InputBuffer.write_cbuf(sample); // get it loaded in!
        uint32_t proc_sample = moving_average(&InputBuffer, &OutputBuffer);
        printf("\nSample: %x \n Processed: %x", sample, proc_sample); // debug via serial
        
        //OutputBuffer.write_cbuf(osc.sin_inc());
        uint out = osc.sin_inc();
        dac_static_write(out);

        sleep_us(100);    


    }
}