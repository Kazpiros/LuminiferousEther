#include "include/dac_spi_config.hpp"

void dac_config_set()
{
    gpio_put(17, 0);    // when do i need to flip these on register config?
    
    uint8_t VREF_m[3] = {0x8<<4,0,0x03};
    spi_write_blocking(SPI_PORT, VREF_m, 3); //VREF address | nil | buffered Vref pin 
    sleep_us(10);
    
    uint8_t PWDN_m[3] = {0x9<<4,0,0x00}; //PWDN address | nil | not powered down
    spi_write_blocking(SPI_PORT, PWDN_m, 3);
    sleep_us(10);

    uint8_t GAIN_m[3] = {0xA<<4,0x01,0x00}; //PWDN address | nil | not powered down
    spi_write_blocking(SPI_PORT, GAIN_m, 3); // GAIN address | gain = 1 | read only
    sleep_us(10);

    gpio_put(17, 1);    // when do i need to flip these on register config?
}

void dac_static_write(uint8_t data)
{
    gpio_put(17, 0);
    spi_write_blocking(SPI_PORT, &data, 1);
    gpio_put(17, 1);
}