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

static uint8_t const DLE = 0x10;
static uint8_t const ETX = 0x03;
static uint8_t const CRC = 0xFF;

static void send_command(uint8_t id, int length, uint8_t const *data) {
    delay(0.5);
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
    usart_set_databits(USART1, 9);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_ODD);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable_rx_interrupt(USART1);

    /* Finally enable the USART. */
    usart_enable(USART1);
    
    send_command(0x0e, 0, nullptr); // disable all
    send_command(0xd7, 2, (uint8_t const []){0x02, 10}); // 10 Hz navigation rate
    send_command(0x27, 1, (uint8_t const []){1}); // PVT
    send_command(0x2a, 1, (uint8_t const []){1}); // ionospheric
    send_command(0x5c, 1, (uint8_t const []){1}); // atmospheric corrections
    send_command(0xd5, 1, (uint8_t const []){1}); // bit information
    send_command(0xf4, 1, (uint8_t const []){1}); // 10 Hz raw data
}

static bool logging_enabled = false;
void gps_start_logging() {
    logging_enabled = true;
}

enum class ParseState : uint8_t {
    WAITING_FOR_DLE,
    WAITING_FOR_ID,
    READING,
    READING_AFTER_DLE,
};
static ParseState parse_state = ParseState::WAITING_FOR_DLE;
static uint8_t packet[1024];
static uint16_t packet_pos;

extern "C" {

__attribute__((interrupt))
void usart1_isr(void) {
    //if(!((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0)) return;
    //if(!((USART_SR(USART1) & USART_SR_RXNE) != 0)) return;
    
    USART_SR(USART1);
    uint8_t data = usart_recv(USART1);
    
    if(packet_pos == sizeof(packet)) {
        // abort if we would overrun
        parse_state = ParseState::WAITING_FOR_DLE;
    }
    
    if(parse_state == ParseState::WAITING_FOR_DLE) {
        if(data == DLE) {
            parse_state = ParseState::WAITING_FOR_ID;
        } else {
            printf("gps: junk %i\n", data);
        }
    } else if(parse_state == ParseState::WAITING_FOR_ID) {
        if(data == ETX || data == DLE || data == CRC) {
            printf("gps: invalid id: %i\n", data);
            parse_state = ParseState::WAITING_FOR_DLE;
        } else {
            packet[0] = data;
            packet_pos = 1;
            parse_state = ParseState::READING;
        }
    } else if(parse_state == ParseState::READING) {
        if(data == DLE) {
            parse_state = ParseState::READING_AFTER_DLE;
        } else {
            packet[packet_pos++] = data;
        }
    } else if(parse_state == ParseState::READING_AFTER_DLE) {
        if(data == DLE) {
            packet[packet_pos++] = DLE;
            parse_state = ParseState::READING;
        } else if(data == ETX) {
            // done!
            // process packet, but remember that we are in an interrupt handler
            
            //printf("gps: success %i %i\n", packet[0], packet_pos);
            
            if(logging_enabled) {
                //cm_disable_interrupts();
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
                //cm_enable_interrupts();
            }
            
            if(packet[0] == 0xF5) {
                double x; memcpy(&x, packet+1, 8);
                int16_t week = packet[9] | (packet[10] << 8);
                printf("time: %i %f\n", week, x);
            }
            
            parse_state = ParseState::WAITING_FOR_DLE;
        } else {
            printf("gps: DLE followed by %i\n", data);
            parse_state = ParseState::WAITING_FOR_DLE;
        }
    }
}

}
