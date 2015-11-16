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
#include "misc.h"

#if defined SYLPHASE_GPSLOG_2A
    #define USART_NUM 3
    #define TX_GPIO_PORT GPIOB
    #define TX_GPIO_PIN GPIO10
    #define RX_GPIO_PORT GPIOB
    #define RX_GPIO_PIN GPIO11
#elif defined SYLPHASE_GPSLOG_2C
    #define USART_NUM 1
    #define TX_GPIO_PORT GPIOB
    #define TX_GPIO_PIN GPIO6
    #define RX_GPIO_PORT GPIOB
    #define RX_GPIO_PIN GPIO7
#else
    #error
#endif

static RunnerBase const *CAT3(usart,USART_NUM,_interrupt_callback) = nullptr;

extern "C" {

void CAT3(usart,USART_NUM,_isr)(void) {
    assert(USART_CR1(CAT2(USART,USART_NUM)) & USART_CR1_TXEIE);
    assert(USART_SR(CAT2(USART,USART_NUM)) & USART_SR_TXE);
    
    usart_disable_tx_interrupt(CAT2(USART,USART_NUM));
    
    assert(CAT3(usart,USART_NUM,_interrupt_callback));
    RunnerBase const *x = CAT3(usart,USART_NUM,_interrupt_callback);
    CAT3(usart,USART_NUM,_interrupt_callback) = nullptr;
    reactor_run_in_main(*x);
}

}

static void my_usart_send_blocking(uint8_t x) {
    if(!(USART_SR(CAT2(USART,USART_NUM)) & USART_SR_TXE)) {
        CoroutineBase * cc = current_coroutine;
        assert(cc);
        auto f = [&]() {
            assert(!cc->run_some());
        };
        Runner<decltype(f)> r(f);
        CAT3(usart,USART_NUM,_interrupt_callback) = &r;
        
        usart_enable_tx_interrupt(CAT2(USART,USART_NUM));
        
        yield();
    }
    assert(USART_SR(CAT2(USART,USART_NUM)) & USART_SR_TXE);
    USART_DR(CAT2(USART,USART_NUM)) = (x & USART_DR_MASK);
}

void serial_setup(void) {
    #ifdef SYLPHASE_GPSLOG_2C
        gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, CAT3(AFIO_MAPR_USART,USART_NUM,_REMAP));
    #endif
    rcc_periph_reset_pulse(CAT2(RST_USART,USART_NUM)); rcc_periph_clock_enable(CAT2(RCC_USART,USART_NUM));
    
    /* Setup GPIO pins */
    gpio_set_mode(TX_GPIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TX_GPIO_PIN);
    gpio_set_mode(RX_GPIO_PORT, GPIO_MODE_INPUT        , GPIO_CNF_INPUT_FLOAT          , RX_GPIO_PIN);

    /* Setup UART parameters. */
    usart_set_baudrate(CAT2(USART,USART_NUM), 115200);
    usart_set_databits(CAT2(USART,USART_NUM), 8);
    usart_set_stopbits(CAT2(USART,USART_NUM), USART_STOPBITS_1);
    usart_set_mode(CAT2(USART,USART_NUM), USART_MODE_TX_RX);
    usart_set_parity(CAT2(USART,USART_NUM), USART_PARITY_NONE);
    usart_set_flow_control(CAT2(USART,USART_NUM), USART_FLOWCONTROL_NONE);
    
    nvic_enable_irq(CAT3(NVIC_USART,USART_NUM,_IRQ));

    /* Finally enable the USART. */
    usart_enable(CAT2(USART,USART_NUM));

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
                usart_send_blocking(CAT2(USART,USART_NUM), '\r');
            }
            usart_send_blocking(CAT2(USART,USART_NUM), ptr[i]);
        }
        return len;
    } else {
        errno = EIO;
        return -1;
    }
}

}
