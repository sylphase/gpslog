#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "gps.h"

#include "time.h"

#include "hardware.h"

static uint8_t const DLE = 0x10;
static uint8_t const ETX = 0x03;
static uint8_t const CRC = 0xFF;

static void send_command(uint8_t id, int length, uint8_t const *data) {
    delay(0.1);
    usart_send_blocking(USART1, DLE);
    usart_send_blocking(USART1, id);
    for(int i = 0; i < length; i++) {
        if(data[i] == DLE) {
            usart_send_blocking(USART1, DLE);
            usart_send_blocking(USART1, DLE);
        } else {
            usart_send_blocking(USART1, data[i]);
        }
    }
    usart_send_blocking(USART1, DLE);
    usart_send_blocking(USART1, ETX);
}

void gps_setup(void) {
    rcc_periph_reset_pulse(RST_USART1); rcc_periph_clock_enable(RCC_USART1);
    
    nvic_enable_irq(NVIC_USART1_IRQ);
    
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
              GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_ODD);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    USART_CR1(USART1) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART1);
    
    send_command(0x0e, 0, nullptr);
    send_command(0xd7, 2, (uint8_t const []){0x02, 10}); // 10 Hz
    send_command(0x27, 1, (uint8_t const []){1});
    send_command(0x2a, 1, (uint8_t const []){1});
    send_command(0x5c, 1, (uint8_t const []){1});
    send_command(0xd5, 1, (uint8_t const []){1});
    send_command(0xf4, 1, (uint8_t const []){1}); // 10 Hz
}

extern "C" {

void usart1_isr(void) {
    set_led_color(LEDColor::RED);
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        char data = usart_recv(USART1);
        usart_send_blocking(USART2, data);
    }
}

}
