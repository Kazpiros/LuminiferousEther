#include "include/adc_spi_config.hpp"

void adc_config_set()
{ 

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

}