#include <cstdio>               // or <stdio.h>
//#include "pico/stdlib.h"
//#include <hardware/spi.h>
#include <cstdint>

//#include "kspi.pio.h"
#include "include/le_defs.h"
#include "blink.pio.h"
#include "serial.pio.h"
//#include "include/adc_spi_config.hpp"

//run with cmake --build build

/**
 * TODO:
 * - Need to add a #define for all CS pins so im not staring at "17"s
 * - Figure out memory leak in Ring buffer
 * - know that non-blocking reads will consume a bunch of processing time 90% is what it feels like.
 * - add a timer for exact sample rate.
 * - 
 * - Setup timer for output frequency control
 * - Make decimation filter to lower rate for DAC
 * - Delay effect + passthru + moving average 
 */



void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

void serial_pio(PIO pio, uint offset, uint pin, uint clkdiv)
{
    serial_program_init(pio, 0, offset, pin, clkdiv);
}


const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];
// Get a free channel, panic() if there are none
int dma_chan = dma_claim_unused_channel(true);

void dma_handler() {
    static bool first_run = true;
    if (first_run) {
        first_run = false;
    }
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan;
    dma_channel_set_read_addr(dma_chan, src, true);
}

int irq_flag = 0;

uint8_t rxbuf[4];
uint8_t the[4] = {0x41,0,0,0};
uint8_t zeros[4] = {0,0,0,0};

void gpio_irq_handler(uint gpio, uint32_t events) { //The GPIO callback function to handle interrupts
    uint32_t sample = (rxbuf[0] << 24) | (rxbuf[1] << 16) | (rxbuf[2] << 8) | rxbuf[3];
    irq_flag = 1;
}

int main()
{

    /** DMA is simple! just needs the TX and RX buffers. Probably good to have two channels. 
     * one for transmit, one for recieve.
     **/
    /***** DMA START *****/ 
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(
        dma_chan,          // Channel to be configured
        &c,            // The configuration we just created
        &pio0_hw->txf[0],// The initial write address. Write address (only need to set this once)
        NULL,           // The initial read address, not providing it yet
        count_of(src),          // Number of transfers; 
        false           // Start until named.
    );
    //dma_channel_wait_for_finish_blocking(dma_chan); // freezes till its done, this is optional.
    /***** DMA END *****/

    /********pio start********/
    PIO pio = pio0;
    //uint offset = pio_add_program(pio, &blink_program);
    uint offset = pio_add_program(pio, &serial_program);
    printf("Loaded program at %d\n", offset);
    // got a frequency of vvv. 4MHz, so w/ no clk div, may have ~40MHz on gpio
    serial_pio(pio, offset, 21 , 10.f); //find an acutal available pin
    
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_handler(); // manually call it to start the transfers & pio program.

    stdio_init_all();
    cbuf InputBuffer(8);
    cbuf OutputBuffer(8);

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
        
        spi_transfer(0b01000001); //0b01101001 static read of address 1010 ->
        temp = spi_write_read_blocking(SPI_PORT, zeros, rxbuf, 4);
        //temp = spi_write_read_blocking(SPI_PORT, the, rxbuf, 4); // clock 4 bytes, reading DATA0..DATA3
        gpio_put(PIN_CS, 1);
        
        uint32_t sample = (rxbuf[0] << 24) | (rxbuf[1] << 16) | (rxbuf[2] << 8) | rxbuf[3];

        sleep_us(10);    


    }
}