#include <cstdio>               // or <stdio.h>
#include <cstdint>

#include "include/le_defs.h"


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

 /* summarizing:
  * Nov 10, 2025,
  * got DMA working, had information be able to be sent to pio to output pin given a "launch command"
  * this has an interrupt request as well. it may work. unsure.
  */

  /* IDEA
  * could have 2 spi functions: static single write, continuous?
  */
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];
// Get a free channel, panic() if there are none
int dma_chan = dma_claim_unused_channel(true);



void dma_handler()
{
    static bool first_run = true;
    if (first_run) {
        first_run = false;
    }
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan;
    int word[] = {0b01000001};
    dma_channel_set_read_addr(dma_chan, word, true);
}
int irq_flag = 0;

uint8_t rxbuf[4];
uint8_t txbuf[4] = {0b01000001,0,0,0};;
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
        NULL,// The initial read address, not providing it yet
        count_of(src),          // Number of transfers; 
        false           // Start until named.
    );
    //dma_channel_wait_for_finish_blocking(dma_chan); // freezes till its done, this is optional.


    /***** DMA END *****/

    /********pio start********/
    PIO pio = pio0;
    dma_channel_set_irq0_enabled(dma_chan, true);

    /* attempt at sharing  line 140 in reference (uart dma)*/
    //irq_add_shared_handler(pio_get_irq_num(pio, 0), pio_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    //pio_set_irqn_source_enabled(pio, 0, pio_get_rx_fifo_not_empty_interrupt_source(pio_sm_rx), true);
    //irq_set_enabled(pio_get_irq_num(pio, 0), true);
    /* attempt at sharing */

    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_handler(); // manually call it to start the transfers & pio program.

    stdio_init_all();

    gpio_init(PIN_IRQ);
    gpio_set_dir(PIN_IRQ, GPIO_IN);
    gpio_pull_down(PIN_IRQ);
    

    int temp = 0;

    dac_config_set();

    gpio_init(PIN_CS);
    gpio_put(PIN_CS, 1);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    pio_spi_inst_t spi = {
            .pio = pio0,
            .sm = 0,
            .cs_pin = PIN_CS
    };
    uint offset = pio_add_program(spi.pio, &spi_cpha1_cs_program);
    printf("Loaded program at %d\n", offset);
    pio_spi_init(spi.pio, spi.sm, offset,
                 8,       // 8 bits per SPI frame
                 31.25f,  // 1 MHz @ 125 clk_sys
                 false,   // CPHA = 0
                 false,   // CPOL = 0
                 PIN_SCK,
                 PIN_MOSI,
                 PIN_MISO
    );
    bi_decl(bi_4pins_with_names(PICO_DEFAULT_SPI_RX_PIN, "SPI RX", PICO_DEFAULT_SPI_TX_PIN, "SPI TX", PICO_DEFAULT_SPI_SCK_PIN, "SPI SCK", PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    //adc_config_set();
    pio_adc_config_set(&spi, spi.cs_pin);
    while(1) // lil test batch
    {
        gpio_put(spi.cs_pin, 0);
        pio_spi_write8_read8_blocking(&spi, txbuf, rxbuf, 1);  //for static read 0b01000001
        gpio_put(spi.cs_pin, 1);
    }

    //sleep_us(10);
    //CONV_START(MUX_VINP_CH0);
    //sleep_us(10);

    while (1) {
        tight_loop_contents();
        
        gpio_put(spi.cs_pin, 0);
        pio_spi_write8_read8_blocking(&spi, txbuf, rxbuf, 1);  //for static read 0b01000001
        gpio_put(spi.cs_pin, 1);

        uint32_t sample = (rxbuf[0] << 24) | (rxbuf[1] << 16) | (rxbuf[2] << 8) | rxbuf[3];
        dma_handler();
        sleep_us(10);    
    }
}