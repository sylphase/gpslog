#include <unistd.h>
#include <errno.h>
#include <cassert>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include "serial.h"
#include "hardware.h"
#include "coroutine.h"
#include "reactor.h"

static RunnerBase const *usart3_interrupt_callback = nullptr;

extern "C" {

void usart3_isr(void) {
    assert(USART_CR1(USART3) & USART_CR1_TXEIE);
    assert(USART_SR(USART3) & USART_SR_TXE);
    
    usart_disable_tx_interrupt(USART3);
    
    assert(usart3_interrupt_callback);
    RunnerBase const *x = usart3_interrupt_callback;
    usart3_interrupt_callback = nullptr;
    reactor_run_in_main(*x);
}

}

static void my_usart_send_blocking(uint8_t x) {
    if(!(USART_SR(USART3) & USART_SR_TXE)) {
        CoroutineBase * cc = current_coroutine;
        assert(cc);
        auto f = [&]() {
            assert(!cc->run_some());
        };
        Runner<decltype(f)> r(f);
        usart3_interrupt_callback = &r;
        
        usart_enable_tx_interrupt(USART3);
        
        yield();
    }
    assert(USART_SR(USART3) & USART_SR_TXE);
    USART_DR(USART3) = (x & USART_DR_MASK);
}

void serial_setup(void) {
    rcc_periph_reset_pulse(RST_USART3); rcc_periph_clock_enable(RCC_USART3);
    
    /* Setup GPIO pin GPIO_USART3_TX. */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
              GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    
    nvic_enable_irq(NVIC_USART3_IRQ);

    /* Finally enable the USART. */
    usart_enable(USART3);

}

extern "C" {

int _write(int file, char *ptr, int len) {
    if(file == STDOUT_FILENO) {
        for(int i = 0; i < len; i++) {
            if(ptr[i] == '\n') {
                my_usart_send_blocking('\r');
            }
            my_usart_send_blocking(ptr[i]);
        }
        return len;
    } else if(file == STDERR_FILENO) {
        for(int i = 0; i < len; i++) {
            if(ptr[i] == '\n') {
                usart_send_blocking(USART3, '\r');
            }
            usart_send_blocking(USART3, ptr[i]);
        }
        return len;
    } else {
        errno = EIO;
        return -1;
    }
}

}
