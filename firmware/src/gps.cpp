#include <cstdio>
#include <cassert>
#include <ctime>
#include <cstring>
#include <cmath>
#include <utility>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include "gps.h"

#include "time.h"
#include "sdcard.h"
#include "main.h"
#include "reactor.h"
#include "coroutine.h"
#include "scheduler.h"
#include "misc.h"

#if defined SYLPHASE_GPSLOG_2A
    #define USART_NUM 1
    #define TX_GPIO_PORT GPIOA
    #define TX_GPIO_PIN GPIO9
    #define RX_GPIO_PORT GPIOA
    #define RX_GPIO_PIN GPIO10
    #define NV08C_CSM
#elif defined SYLPHASE_GPSLOG_2C
    #define USART_NUM 2
    #define TX_GPIO_PORT GPIOA
    #define TX_GPIO_PIN GPIO2
    #define RX_GPIO_PORT GPIOA
    #define RX_GPIO_PIN GPIO3
    #define S1315F8_RAW
#else
    #error
#endif

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

void CAT3(usart,USART_NUM,_isr)(void) {
    USART_SR(CAT2(USART,USART_NUM));
    if(!((USART_SR(CAT2(USART,USART_NUM)) & USART_SR_RXNE) != 0)) return;
    uint8_t data = usart_recv(CAT2(USART,USART_NUM));
    
    if(!((USART_CR1(CAT2(USART,USART_NUM)) & USART_CR1_RXNEIE) != 0)) return;
    
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



static bool logging_enabled = false;
void gps_start_logging() {
    logging_enabled = true;
}


#if defined NV08C_CSM

static uint8_t const DLE = 0x10;
static uint8_t const ETX = 0x03;
static uint8_t const CRC = 0xFF;

static void send_command(uint8_t id, int length, uint8_t const *data) {
    yield_delay(0.5);
    usart_send_blocking(CAT2(USART,USART_NUM), DLE);
    usart_send_blocking(CAT2(USART,USART_NUM), id);
    for(int i = 0; i < length; i++) {
        if(data[i] == DLE) {
            usart_send_blocking(CAT2(USART,USART_NUM), DLE);
            usart_send_blocking(CAT2(USART,USART_NUM), DLE);
        } else {
            usart_send_blocking(CAT2(USART,USART_NUM), data[i]);
        }
    }
    usart_send_blocking(CAT2(USART,USART_NUM), DLE);
    usart_send_blocking(CAT2(USART,USART_NUM), ETX);
}

bool gps_write_packet(uint8_t const * data, uint32_t length) {
    assert(length >= 1);
    assert(data[0] != ETX && data[0] != DLE && data[0] != CRC);
    
    uint32_t bytes_required = 1 + length + 2;
    for(uint32_t i = 1; i < length; i++) {
        if(data[i] == DLE) bytes_required++;
    }
    
    if(sdcard_buf.write_available() >= bytes_required) {
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

#elif defined S1315F8_RAW

static void send_command(int length, uint8_t const *data) {
    yield_delay(0.5);
    usart_send_blocking(CAT2(USART,USART_NUM), 0xA0);
    usart_send_blocking(CAT2(USART,USART_NUM), 0xA1);
    usart_send_blocking(CAT2(USART,USART_NUM), length >> 8);
    usart_send_blocking(CAT2(USART,USART_NUM), length & 0xff);
    uint8_t checksum = 0;
    for(int i = 0; i < length; i++) {
        usart_send_blocking(CAT2(USART,USART_NUM), data[i]);
        checksum ^= data[i];
    }
    usart_send_blocking(CAT2(USART,USART_NUM), checksum);
    usart_send_blocking(CAT2(USART,USART_NUM), 0x0D);
    usart_send_blocking(CAT2(USART,USART_NUM), 0x0A);
}

bool gps_write_packet(uint8_t const * data, uint32_t length) {
    uint32_t bytes_required = 2 + 2 + length + 1 + 2;
    
    if(sdcard_buf.write_available() >= bytes_required) {
        assert(sdcard_buf.write_one(0xA0));
        assert(sdcard_buf.write_one(0xA1));
        assert(sdcard_buf.write_one(length >> 8));
        assert(sdcard_buf.write_one(length & 0xff));
        uint8_t checksum = 0;
        for(uint16_t i = 0; i < length; i++) {
            assert(sdcard_buf.write_one(data[i]));
            checksum ^= data[i];
        }
        assert(sdcard_buf.write_one(checksum));
        assert(sdcard_buf.write_one(0x0D));
        assert(sdcard_buf.write_one(0x0A));
        
        return true;
    } else {
        return false;
    }
}

static void gps_coroutine_function() {
    bool called_got_date_string = false;
    
    while(true) {
start:
        uint8_t current_byte;
        uint8_t packet[1024];
        current_byte = get_byte(); if(current_byte != 0xA0)  {
            my_printf("gps: junk: %i\n", current_byte);
            goto start;
        }
        current_byte = get_byte(); if(current_byte != 0xA1)  {
            my_printf("gps: junk: %i\n", current_byte);
            goto start;
        }
        
        current_byte = get_byte();
        uint16_t packet_length = (current_byte << 8) | get_byte();
        if(packet_length == 0) {
            my_printf("gps: zero length packet\n");
            goto start;
        }
        if(packet_length > sizeof(packet)) {
            my_printf("gps: packet too long: %i\n", packet_length);
            goto start;
        }
        
        uint8_t expected_checksum = 0;
        for(uint16_t i = 0; i < packet_length; i++) {
            uint8_t b = get_byte();
            packet[i] = b;
            expected_checksum ^= b;
        }
        
        uint8_t checksum = get_byte();
        if(checksum != expected_checksum) {
            my_printf("gps: invalid checksum\n");
            goto start;
        }
        
        current_byte = get_byte(); if(current_byte != 0x0D)  {
            my_printf("gps: junk: %i\n", current_byte);
            goto start;
        }
        current_byte = get_byte(); if(current_byte != 0x0A)  {
            my_printf("gps: junk: %i\n", current_byte);
            goto start;
        }
        
        my_printf("gps: success %i %i\n", packet[0], packet_length);
        
        if(logging_enabled) {
            gps_write_packet(packet, packet_length); // silently drop if buffer is full
        }
        
        if(packet[0] == 0xDF && !called_got_date_string && packet[2] >= 2) {
            uint16_t week = (packet[3] << 8) | packet[4];
            uint8_t x_int[8]; memcpy(&x_int, packet+5, 8);
            for(int i = 0; i < 3; i++) std::swap(x_int[i], x_int[7-i]);
            double x; memcpy(&x, x_int, 8);
            double s = 315964800 + 24*60*60*7 * (week) + x;
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

#else
    #error
#endif

static Coroutine<3072> gps_coroutine;
void gps_setup(void) {
    rcc_periph_reset_pulse(CAT2(RST_USART,USART_NUM)); rcc_periph_clock_enable(CAT2(RCC_USART,USART_NUM));
    
    gpio_set_mode(TX_GPIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TX_GPIO_PIN);
    gpio_set_mode(RX_GPIO_PORT, GPIO_MODE_INPUT        , GPIO_CNF_INPUT_FLOAT          , RX_GPIO_PIN);

    /* Setup UART parameters. */
    usart_set_mode(CAT2(USART,USART_NUM), USART_MODE_TX_RX);
    usart_set_baudrate(CAT2(USART,USART_NUM), 115200);
    usart_set_flow_control(CAT2(USART,USART_NUM), USART_FLOWCONTROL_NONE);
    usart_set_stopbits(CAT2(USART,USART_NUM), USART_STOPBITS_1);
    #if defined NV08C_CSM
        usart_set_databits(CAT2(USART,USART_NUM), 9);
        usart_set_parity(CAT2(USART,USART_NUM), USART_PARITY_ODD);
    #elif defined S1315F8_RAW
        usart_set_databits(CAT2(USART,USART_NUM), 8);
        usart_set_parity(CAT2(USART,USART_NUM), USART_PARITY_NONE);
    #else
        #error
    #endif

    /* Finally enable the USART. */
    usart_enable(CAT2(USART,USART_NUM));
    
    #if defined NV08C_CSM
        send_command(0x0e, 0, nullptr); // disable all
        send_command(0xd7, 2, (uint8_t const []){0x02, 10}); // 10 Hz navigation rate
        send_command(0x27, 1, (uint8_t const []){1}); // PVT
        send_command(0x2a, 1, (uint8_t const []){1}); // ionospheric
        send_command(0x5c, 1, (uint8_t const []){1}); // atmospheric corrections
        send_command(0xd5, 1, (uint8_t const []){1}); // bit information
        send_command(0xf4, 1, (uint8_t const []){1}); // 10 Hz raw data
    #elif defined S1315F8_RAW
        send_command(3, (uint8_t const []){0x09,  2, 0}); // binary out
        send_command(3, (uint8_t const []){0x0E, 10, 0}); // position rate
        send_command(8, (uint8_t const []){0x1E, 4, 1, 1, 1, 1, 0x0F, 0}); // binary measurement rates
        send_command(2, (uint8_t const []){0x30, 0}); // get ephemerides
    #else
        #error
    #endif
    
    nvic_enable_irq(CAT3(NVIC_USART,USART_NUM,_IRQ));
    usart_enable_rx_interrupt(CAT2(USART,USART_NUM));
    
    gps_coroutine.start(gps_coroutine_function);
}
