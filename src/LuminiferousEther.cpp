#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include <stdint.h>

#include "ADC_SPI/MCP3x6x_Type_Definitions.h"                                                           // Variable definitions header-file.
#include "ADC_SPI/MCP3x6x_ADC_Definitions.h"                                                            // ADC definitions header-file.
#include "ADC_SPI/MCP3x6x_Peripheral_Definitions.h"                                                     // Peripheral definitions header-file.
#include "ADC_SPI/MCP3x6x_SPI_Definitions.h"  

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];


int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

uint8_t zeros[4] = {0,0,0,0};
uint8_t rxbuf[4];

volatile unsigned long long int counter = 0; //The counter variable

void gpio_callback(uint gpio, uint32_t events) { //The GPIO callback function to handle interrupts
    uint32_t sample = (rxbuf[0] << 24) | (rxbuf[1] << 16) | (rxbuf[2] << 8) | rxbuf[3];
    printf("0x%08X\n", sample);
}


int main()
{
    

    stdio_init_all();
    gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    //we use spi mode 1,1
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    //gpio_set_dir(20, GPIO_IN); //IRQ

    /* ######## start of MCP ######## */
    //MCP3x6x_CONFIG();
    /* ######## start of MCP ######## */

    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // Get a free channel, panic() if there are none
    int chan = dma_claim_unused_channel(true);
    
    // 8 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.
    
    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    
    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        dst,           // The initial write address
        src,           // The initial read address
        count_of(src), // Number of transfers; in this case each is 1 byte.
        true           // Start immediately.
    );
    
    // We could choose to go and do something else whilst the DMA is doing its
    // thing. In this case the processor has nothing else to do, so we just
    // wait for the DMA to finish.
    dma_channel_wait_for_finish_blocking(chan);
    
    // The DMA has now copied our text from the transmit buffer (src) to the
    // receive buffer (dst), so we can print it out from there.
    puts(dst);

    // Interpolator example code
    //interp_config cfg = interp_default_config();
    // Now use the various interpolator library functions for your use case
    // e.g. interp_config_clamp(&cfg, true);
    //      interp_config_shift(&cfg, 2);
    // Then set the config 
    //interp_set_config(interp0, 0, &cfg);
    // For examples of interpolator use see https://github.com/raspberrypi/pico-examples/tree/master/interp

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks


    _u32data_t CONV_DATA; 
    CONV_DATA.DWORD = 0x00000000;

    _u32data_t TEMP_SNSR_DATA;
    TEMP_SNSR_DATA.DWORD = 0x00000000;

    _u16data_t ADC_CRC;
    ADC_CRC.WORD = 0x0000;

    uint64_t CALC_CRC = 0x000000000000;
    
    int K_addr = 0b01000000;

    uint8_t cmd_static_adCDATA = 0x41; // 0b01 0000 01 -> device addr=01, addr=0x0, static read

    gpio_put(PIN_CS, 0);                 // CS low -> begin SPI communication
    spi_write_blocking(SPI_PORT, &cmd_static_adCDATA, 1); // send COMMAND byte; STATUS will be clocked out now
    gpio_put(PIN_CS, 1);  
    sleep_us(10);
    CONV_START(MUX_VINP_CH0);
    sleep_us(10);
    // Now continuously clock SCK to stream ADC data.  Read in blocks matching your data width (e.g. 4 bytes for 32-bit).
    
    //MCP3x6x_CONFIG();

    gpio_put(17, 0);    // when do i need to flip these on register config?
    //make sure set all reserved bits?!
    //config start
    spi_transfer(0b01000010);// | 0b00000100 | 0b00000010); // Asking device address | regaddress start | incremental write

    gpio_put(17, 1); 
    sleep_us(10);
    //config0
    SPI_WRT(_CONFIG0_,CLK_SEL_INTOSC_NOCLKOUT | CS_SEL_NONE | ADC_MODE_CONV); // change later when mclk is figured out.. **what is conversion mode**
    sleep_us(10);
    //config1
    SPI_WRT(_CONFIG1_,PRE_MCLK | OSR_32 | 0x00); // no prescaler from internal or external clock, lowest OSR
    sleep_us(10);
    //config2
    SPI_WRT(_CONFIG2_,BOOST_1X | GAIN_1X | AZ_MUX_OFF | 0x3); // all default (with a 11 on the last two bits for reserved)
    sleep_us(10);
    //config3
    SPI_WRT(_CONFIG3_,CONV_MODE_CONT | DATA_FRMT_32b25b_CHID | CRC_FRMT_16b | EN_CRCCOM_OFF | EN_OFFCAL_OFF | EN_GAINCAL_OFF); //continous conversion | sign extension + 16bits | no crc | defaults..
    sleep_us(10);
    //IRQ
    SPI_WRT(_IRQ_,IRQ_MODE1_IRQ | IRQ_MODE0_HI | EN_FASTCMD_ON | EN_STP_ON); // IRQ output | no irq pullup required | enable fast cmds | enable conversion start output
    sleep_us(10);
    //MUX 
    SPI_WRT(_MUX_,MUX_VINP_CH0 | MUX_VINN_CH1); // channel 0 and 1 for v+ and v-
    sleep_us(10);
    //SCAN (24 bit register)
    SPI_WRT(_SCAN_, SCAN_CFG);// go to table 5-14 for scan and scan channel selection I DONT KNOW WHAT THIS IS
    sleep_us(10);
    //TIMER 24 bit
    SPI_WRT(_TIMER_,TIMER_CFG); // no scan delay
    sleep_us(10);
    //OFFSETCAL 24 bit
    SPI_WRT(_OFFSETCAL_,OFFSETCAL_CFG); // look at 5.12 for calibration of this (maybe just for default settings.)
    sleep_us(10);
    //GAINCAL 24 bit
    SPI_WRT(_GAINCAL_,GAINCAL_CFG); // look at 5.12 for calibration of this (maybe just for default settings.)
    sleep_us(10);
    //LOCK
    SPI_WRT(_LOCK_,LOCK_CFG); //allows full write access, put this first if funkyness is happening
    sleep_us(10);

    //
    CONV_START(MUX_VINP_CH0);
    //

    int temp = 0;
    while (true) {
        gpio_put(PIN_CS, 0);   
        spi_transfer(0b01000001); //0b01101001 static read of address 1010 ->
        temp = spi_write_read_blocking(SPI_PORT, zeros, rxbuf, 4); // clock 4 bytes, reading DATA0..DATA3
        gpio_put(PIN_CS, 1);
        sleep_us(10);    
    }
}