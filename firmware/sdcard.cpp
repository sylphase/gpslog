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

static uint8_t CRC[256] = {0, 9, 18, 27, 36, 45, 54, 63, 72, 65, 90, 83, 108, 101, 126, 119, 25, 16, 11, 2, 61, 52, 47, 38, 81, 88, 67, 74, 117, 124, 103, 110, 50, 59, 32, 41, 22, 31, 4, 13, 122, 115, 104, 97, 94, 87, 76, 69, 43, 34, 57, 48, 15, 6, 29, 20, 99, 106, 113, 120, 71, 78, 85, 92, 100, 109, 118, 127, 64, 73, 82, 91, 44, 37, 62, 55, 8, 1, 26, 19, 125, 116, 111, 102, 89, 80, 75, 66, 53, 60, 39, 46, 17, 24, 3, 10, 86, 95, 68, 77, 114, 123, 96, 105, 30, 23, 12, 5, 58, 51, 40, 33, 79, 70, 93, 84, 107, 98, 121, 112, 7, 14, 21, 28, 35, 42, 49, 56, 65, 72, 83, 90, 101, 108, 119, 126, 9, 0, 27, 18, 45, 36, 63, 54, 88, 81, 74, 67, 124, 117, 110, 103, 16, 25, 2, 11, 52, 61, 38, 47, 115, 122, 97, 104, 87, 94, 69, 76, 59, 50, 41, 32, 31, 22, 13, 4, 106, 99, 120, 113, 78, 71, 92, 85, 34, 43, 48, 57, 6, 15, 20, 29, 37, 44, 55, 62, 1, 8, 19, 26, 109, 100, 127, 118, 73, 64, 91, 82, 60, 53, 46, 39, 24, 17, 10, 3, 116, 125, 102, 111, 80, 89, 66, 75, 23, 30, 5, 12, 51, 58, 33, 40, 95, 86, 77, 68, 123, 114, 105, 96, 14, 7, 28, 21, 42, 35, 56, 49, 70, 79, 84, 93, 98, 107, 112, 121};

static uint8_t CMD(uint8_t n, uint32_t argument, uint32_t * ocr=nullptr) {
    assert(n <= 63);
    uint8_t crc = 0;
#define SEND(b) { \
    uint8_t _ = (b); \
    spi_xfer(SPI1, _); \
    crc = CRC[(crc << 1) ^ _]; \
}
    gpio_clear(GPIOA, GPIO15);
    SEND(0b01000000 | n);
    SEND((argument >> 24) & 255);
    SEND((argument >> 16) & 255);
    SEND((argument >>  8) & 255);
    SEND((argument >>  0) & 255);
    spi_xfer(SPI1, (crc << 1) | 1);
    uint16_t resp;
    while(true) { // should abort after 8 bytes (N_CR)
        resp = spi_xfer(SPI1, 0xFF);
        if((resp & 0b10000000) == 0) {
            break;
        }
    }
    if(ocr) {
        *ocr = 0;
        for(int i = 0; i < 4; i++) {
            *ocr <<= 8;
            *ocr |= spi_xfer(SPI1, 0xFF);
        }
    }
    gpio_set(GPIOA, GPIO15);
    delay(1e-6);
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
    
    assert(CMD(0, 0) == 1); // reset, in idle state now
    
    // XXX code after here only supports SD Ver.2
    uint32_t CMD8_aux; assert(CMD(8, 0x1AA, &CMD8_aux) == 1); // check voltage
    assert(CMD8_aux == 0x1AA);
    
    assert(CMD(55, 0) == 1); // ACMD prefix
    
    assert(CMD(41, 0x40000000) == 1); // ACMD41(0x40000000)
    
    uint32_t CMD58_aux; assert(CMD(58, 0, &CMD58_aux) == 1); // read ocr
    
    if(CMD58_aux & (1<<30)) {
        byte_addresses = false;
    } else {
        byte_addresses = true;
        
        assert(CMD(16, 0x200) == 1); // set block size to 512 bytes
    }
    
    printf("byte_addresses: %s\n", byte_addresses ? "true" : "false");
    
    printf("done\n");
}
