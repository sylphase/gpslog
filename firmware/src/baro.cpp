#include <cstdio>
#include <cassert>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "baro.h"
#include "coroutine.h"
#include "time.h"
#include "reactor.h"

/*
        PB12=baro_spi_nCS, # SPI2_NSS
        PB13=baro_spi.SCLK, # SPI2_SCK
        PB14=baro_spi.MISO, # SPI2_MISO
        PB15=baro_spi.MOSI, # SPI2_MOSI
*/

static void send_command(uint8_t cmd, uint32_t read=0, uint8_t * dest=nullptr) {
    gpio_clear(GPIOB, GPIO12);
    SPI_DR(SPI2) = cmd; yield();
    while(read--) {
        SPI_DR(SPI2) = cmd; yield(); *dest++ = SPI_DR(SPI2);
    }
    gpio_set(GPIOB, GPIO12);
}

static Coroutine<512> baro_coroutine;

static void baro_main() {
    uint16_t prom[8];
    send_command(0x1E); // Reset
    delay2(15e-3);
    for(int i = 0; i < 8; i++) {
        uint8_t buf[2];
        send_command(0xA0 + 2 * i, 2, buf); // PROM Read
        prom[i] = (buf[0] << 8) | buf[1];
        printf("prom[%i] = %i\n", i, prom[i]);
    }
    while(true) delay2(1);
}

void baro_init() {
    rcc_periph_reset_pulse(RST_SPI2); rcc_periph_clock_enable(RCC_SPI2);
    
    gpio_set(GPIOB, GPIO12);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13);
    gpio_set(GPIOB, GPIO14); // pullup
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
    
    spi_reset(SPI2);
    assert(rcc_apb1_frequency / 2 <= 20e6);
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);
    
    spi_enable(SPI2);
    
    nvic_enable_irq(NVIC_SPI2_IRQ);
    spi_enable_rx_buffer_not_empty_interrupt(SPI2);
    
    baro_coroutine.start(baro_main);
}

static void got_byte(void *, uint32_t) {
    assert(!baro_coroutine.run_some());
}

extern "C" {

void spi2_isr(void) {
    assert(SPI_CR2(SPI2) & SPI_CR2_RXNEIE);
    assert(SPI_SR(SPI2) & SPI_SR_RXNE);
    
    SPI_DR(SPI2); // clear interrupt
    
    assert(main_callbacks.write_one(CallbackRecord(got_byte, nullptr, 0)));
}

}
