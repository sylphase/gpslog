#include <stdio.h>

#include <cassert>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

#include "sdcard.h"
#include "time.h"

/*
        PA15=sd_spi_nCS, # SPI1_NSS
        PB3=sd_spi.SCLK, # SPI1_SCK
        PB4=sd_spi.MISO, # SPI1_MISO
        PB5=sd_spi.MOSI, # SPI1_MOSI
*/

static uint8_t CMD_R1(uint8_t n, uint32_t argument, uint8_t crc=0) {
    assert(n <= 63);
    spi_xfer(SPI1, 0b01000000 | n);
    spi_xfer(SPI1, (argument >> 24) & 255);
    spi_xfer(SPI1, (argument >> 16) & 255);
    spi_xfer(SPI1, (argument >>  8) & 255);
    spi_xfer(SPI1, (argument >>  0) & 255);
    spi_xfer(SPI1, (crc << 1) | 1);
    uint16_t resp;
    while(true) { // should abort after 8 bytes (N_CR)
        resp = spi_xfer(SPI1, 0xFF);
        if((resp & 0b10000000) == 0) {
            break;
        }
    }
    return resp;
}

static uint8_t CMD_R3_or_R7(uint8_t n, uint32_t argument, uint32_t & ocr, uint8_t crc=0) {
    assert(n <= 63);
    spi_xfer(SPI1, 0b01000000 | n);
    spi_xfer(SPI1, (argument >> 24) & 255);
    spi_xfer(SPI1, (argument >> 16) & 255);
    spi_xfer(SPI1, (argument >>  8) & 255);
    spi_xfer(SPI1, (argument >>  0) & 255);
    spi_xfer(SPI1, (crc << 1) | 1);
    uint16_t resp;
    while(true) { // should abort after 8 bytes (N_CR)
        resp = spi_xfer(SPI1, 0xFF);
        if((resp & 0b10000000) == 0) {
            break;
        }
    }
    ocr = 0;
    for(int i = 0; i < 4; i++) {
        ocr <<= 8;
        ocr |= spi_xfer(SPI1, 0xFF);
    }
    return resp;
}

bool byte_addresses;

void sdcard_init() {
    printf("hello world from sdcard!\n");
    
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI1_REMAP);
    
    rcc_periph_reset_pulse(RST_SPI1); rcc_periph_clock_enable(RCC_SPI1);
    
    gpio_set(GPIOA, GPIO15);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
    
    //gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO3);
    gpio_set(GPIOB, GPIO4); // pullup
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);
    
    spi_reset(SPI1);
    assert(rcc_apb1_frequency / 256 >= 100e3 && rcc_apb1_frequency / 256 <= 400e3);
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_set_full_duplex_mode(SPI1);
    
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    
    spi_enable(SPI1);
    
    // implemented referencing http://elm-chan.org/docs/mmc/mmc_e.html
    
    delay(1e-3);
    
    for(int i = 0; i < (74+7)/8; i++) spi_xfer(SPI1, 0xFF);
    
    gpio_clear(GPIOA, GPIO15);
    assert(CMD_R1(0, 0, 0b1001010) == 1); // reset, in idle state now
    gpio_set(GPIOA, GPIO15);
    delay(1e-6);
    
    gpio_clear(GPIOA, GPIO15);
    uint32_t CMD8_aux;
    // XXX code after here only supports SD Ver.2
    assert(CMD_R3_or_R7(8, 0x1AA, CMD8_aux, 0b1000011) == 1); // check voltage
    assert(CMD8_aux == 0x1AA);
    gpio_set(GPIOA, GPIO15);
    delay(1e-6);
    
    gpio_clear(GPIOA, GPIO15);
    assert(CMD_R1(55, 0) == 1); // ACMD prefix
    gpio_set(GPIOA, GPIO15);
    delay(1e-6);
    
    gpio_clear(GPIOA, GPIO15);
    assert(CMD_R1(41, 0x40000000) == 1); // ACMD41(0x40000000)
    gpio_set(GPIOA, GPIO15);
    delay(1e-6);
    
    gpio_clear(GPIOA, GPIO15);
    uint32_t CMD58_aux;
    assert(CMD_R3_or_R7(58, 0, CMD58_aux) == 1); // read ocr
    gpio_set(GPIOA, GPIO15);
    delay(1e-6);
    
    if(CMD58_aux & (1<<30)) {
        byte_addresses = false;
    } else {
        byte_addresses = true;
        
        gpio_clear(GPIOA, GPIO15);
        assert(CMD_R1(16, 0x200) == 1); // set block size to 512 bytes
        gpio_set(GPIOA, GPIO15);
        delay(1e-6);
    }
    
    printf("byte_addresses: %s\n", byte_addresses ? "true" : "false");
    
    printf("done\n");
}
