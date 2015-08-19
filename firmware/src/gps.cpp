#include <cstdio>
#include <cassert>
#include <ctime>
#include <cstring>
#include <cmath>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include "gps.h"

#include "time.h"
#include "hardware.h"
#include "sdcard.h"
#include "main.h"
#include "reactor.h"
#include "coroutine.h"
#include "scheduler.h"
#include "misc.h"


static CircularBuffer<uint8_t, 128> in_buf;
static CoroutineBase *coroutine_waiting_for_byte = nullptr;
static volatile bool callback_queued;

static void got_interrupt() {
    assert(coroutine_waiting_for_byte);
    CoroutineBase *x = coroutine_waiting_for_byte;
    coroutine_waiting_for_byte = nullptr;
    assert(!x->run_some());
}
static Runner<decltype(got_interrupt)> got_interrupt_runner(got_interrupt);


extern "C" {

void usart1_isr(void) {
    USART_SR(USART1);
    if(!((USART_SR(USART1) & USART_SR_RXNE) != 0)) return;
    uint8_t data = usart_recv(USART1);
    
    if(!((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0)) return;
    
    in_buf.write_one(data); // wiil silently drop bytes if buffer is full
    
    if(coroutine_waiting_for_byte && !callback_queued) {
        reactor_run_in_main(got_interrupt_runner);
        callback_queued = true;
    }
}

}

static uint8_t get_byte() {
    bool do_yield = false;
    { CriticalSection cs;
        if(!in_buf.read_available()) {
            assert(!coroutine_waiting_for_byte);
            coroutine_waiting_for_byte = current_coroutine;
            callback_queued = false;
            do_yield = true;
        }
    }
    
    if(do_yield) {
        yield();
        
        assert(in_buf.read_available());
    }
    
    uint8_t x;
    assert(in_buf.read_one(x));
    return x;
}




static uint8_t const DLE = 0x10;
static uint8_t const ETX = 0x03;
static uint8_t const CRC = 0xFF;

static void send_command(uint8_t id, int length, uint8_t const *data) {
    yield_delay(0.5);
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

static bool logging_enabled = false;
void gps_start_logging() {
    logging_enabled = true;
}

bool gps_write_packet(uint8_t const * data, uint32_t length) {
    uint32_t bytes_required = length;
    for(uint32_t i = 1; i < length; i++) {
        if(data[i] == DLE) bytes_required++;
    }
    
    if(sdcard_buf.write_available() >= 2*length) {
        assert(sdcard_buf.write_one(DLE));
        assert(sdcard_buf.write_one(data[0]));
        for(uint16_t i = 1; i < length; i++) {
            if(data[i] == DLE) {
                assert(sdcard_buf.write_one(DLE));
                assert(sdcard_buf.write_one(DLE));
            } else {
                assert(sdcard_buf.write_one(data[i]));
            }
        }
        assert(sdcard_buf.write_one(DLE));
        assert(sdcard_buf.write_one(ETX));
        
        return true;
    } else {
        return false;
    }
}

static void gps_coroutine_function() {
    uint8_t packet[1024];
    uint16_t packet_pos;
    bool called_got_date_string = false;
    uint8_t current_byte;
    
    while(true) {
start:
        current_byte = get_byte(); if(current_byte != DLE)  {
            my_printf("gps: junk: %i\n", current_byte);
            goto start;
        }
        current_byte = get_byte();
        if(current_byte == ETX || current_byte == DLE || current_byte == CRC) {
            my_printf("gps: invalid id: %i\n", current_byte);
            goto start;
        }
        packet[0] = current_byte;
        packet_pos = 1;
        while(true) {
            current_byte = get_byte();
            if(current_byte == DLE) {
                current_byte = get_byte();
                if(current_byte == DLE) {
                    if(packet_pos == sizeof(packet)) {
                        my_printf("gps: packet too long\n");
                        goto start;
                    }
                    packet[packet_pos++] = DLE;
                } else if(current_byte == ETX) {
                    break;
                } else {
                    my_printf("gps: DLE followed by %i\n", current_byte);
                    goto start;
                }
            } else {
                if(packet_pos == sizeof(packet)) {
                    my_printf("gps: packet too long\n");
                    goto start;
                }
                packet[packet_pos++] = current_byte;
            }
        }
        
        // process packet
        my_printf("gps: success %i %i\n", packet[0], packet_pos);
        
        if(logging_enabled) {
            gps_write_packet(packet, packet_pos); // silently drop if buffer is full
        }
        
        if(packet[0] == 0xF5 && !called_got_date_string) {
            double x; memcpy(&x, packet+1, 8); // XXX endianness
            int16_t week = packet[9] | (packet[10] << 8);
            double s = 315964800 + 24*60*60*7 * (1024+week) + x/1000;
            std::time_t t;
            t = s;
            std::tm const * tm = gmtime(&t);
            assert(tm);
            char date[100];
            strftime(date, sizeof(date), "%Y%m%d-%H%M%S", tm);
            my_printf("time: %s\n", date);
            got_date_string(date);
            called_got_date_string = true;
        }
    }
}

static Coroutine<3072> gps_coroutine;
void gps_setup(void) {
    rcc_periph_reset_pulse(RST_USART1); rcc_periph_clock_enable(RCC_USART1);
    
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
              GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 9);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_ODD);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);
    
    send_command(0x0e, 0, nullptr); // disable all
    send_command(0xd7, 2, (uint8_t const []){0x02, 10}); // 10 Hz navigation rate
    send_command(0x27, 1, (uint8_t const []){1}); // PVT
    send_command(0x2a, 1, (uint8_t const []){1}); // ionospheric
    send_command(0x5c, 1, (uint8_t const []){1}); // atmospheric corrections
    send_command(0xd5, 1, (uint8_t const []){1}); // bit information
    send_command(0xf4, 1, (uint8_t const []){1}); // 10 Hz raw data
    
    nvic_enable_irq(NVIC_USART1_IRQ);
    usart_enable_rx_interrupt(USART1);
    
    gps_coroutine.start(gps_coroutine_function);
}
