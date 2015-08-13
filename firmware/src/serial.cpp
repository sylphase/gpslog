#include <unistd.h>
#include <errno.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "serial.h"
#include "hardware.h"

void serial_setup(void) {
    rcc_periph_reset_pulse(RST_USART2); rcc_periph_clock_enable(RCC_USART2);
    
    /* Setup GPIO pin GPIO_USART2_TX. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
              GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 9);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_ODD);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    
    nvic_enable_irq(NVIC_USART2_IRQ);
    usart_enable_rx_interrupt(USART2);

    /* Finally enable the USART. */
    usart_enable(USART2);

}

CircularBuffer<uint8_t, 128> serial_buf;

extern "C" {

int _write(int file, char *ptr, int len) {
    if(file == STDOUT_FILENO || file == STDERR_FILENO) {
        for(int i = 0; i < len; i++) {
            if(ptr[i] == '\n') {
                usart_send_blocking(USART2, '\r');
            }
            usart_send_blocking(USART2, ptr[i]);
        }
        return len;
    }
    errno = EIO;
    return -1;
}

void usart2_isr(void) {
    //set_led_color(LEDColor::RED);
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        char data = usart_recv(USART2);
        serial_buf.write_one(data);
    }
}

}
