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
}

static bool logging_enabled = false;
void gps_start_logging() {
    logging_enabled = true;
}

static uint8_t packet[1024];
static uint16_t packet_pos;

static bool called_got_date_string = false;

volatile static uint8_t current_byte;

void parse_coroutine_function() {
    // this coroutine doesn't currently use the reactor, so printf/any other yielding functions aren't allowed
    while(true) {
start:
        yield(); if(current_byte != DLE)  {
            //fprintf(stderr, "gps: junk: %i\n", current_byte);
            goto start;
        }
        yield();
        if(current_byte == ETX || current_byte == DLE || current_byte == CRC) {
            //fprintf(stderr, "gps: invalid id: %i\n", current_byte);
            goto start;
        }
        packet[0] = current_byte;
        packet_pos = 1;
        while(true) {
            yield();
            if(current_byte == DLE) {
                yield();
                if(current_byte == DLE) {
                    if(packet_pos == sizeof(packet)) {
                        //fprintf(stderr, "gps: packet too long\n");
                        goto start;
                    }
                    packet[packet_pos++] = DLE;
                } else if(current_byte == ETX) {
                    break;
                } else {
                    //fprintf(stderr, "gps: DLE followed by %i\n", current_byte);
                    goto start;
                }
            } else {
                if(packet_pos == sizeof(packet)) {
                    //fprintf(stderr, "gps: packet too long\n");
                    goto start;
                }
                packet[packet_pos++] = current_byte;
            }
        }
        
        // process packet
        //fprintf(stderr, "gps: success %i %i\n", packet[0], packet_pos);
        
        if(logging_enabled) {
            if(sdcard_buf.write_available() >= 2*packet_pos) { // drop otherwise
                assert(sdcard_buf.write_one(DLE));
                assert(sdcard_buf.write_one(packet[0]));
                for(uint16_t i = 1; i < packet_pos; i++) {
                    if(packet[i] == DLE) {
                        assert(sdcard_buf.write_one(DLE));
                        assert(sdcard_buf.write_one(DLE));
                    } else {
                        assert(sdcard_buf.write_one(packet[i]));
                    }
                }
                assert(sdcard_buf.write_one(DLE));
                assert(sdcard_buf.write_one(ETX));
            }
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
            //fprintf(stderr, "time: %s\n", date);
            got_date_string(date);
            called_got_date_string = true;
        }
    }
}
static Coroutine<1024> parse_coroutine(parse_coroutine_function);

static void got_byte(void *, uint32_t data2) {
    current_byte = data2;
    assert(!parse_coroutine.run_some());
}

extern "C" {

void usart1_isr(void) {
    USART_SR(USART1);
    if(!((USART_SR(USART1) & USART_SR_RXNE) != 0)) return;
    uint8_t data = usart_recv(USART1);
    
    if(!((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0)) return;
    
    main_callbacks.write_one(CallbackRecord(got_byte, nullptr, data));
}

}
