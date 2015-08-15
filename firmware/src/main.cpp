#include <cstdio>
#include <math.h>
#include <unistd.h>
#include <cstring>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/cortex.h>

#include "main.h"

#include "hardware.h"
#include "time.h"
#include "serial.h"
#include "gps.h"
#include "sdcard.h"
#include "coroutine.h"

static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    
    rcc_periph_reset_pulse(RST_GPIOA); rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_reset_pulse(RST_GPIOB); rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_reset_pulse(RST_AFIO); rcc_periph_clock_enable(RCC_AFIO);
}

extern "C" {

void _exit(int status) {
    // called on assertion failure - blink red/green to indicate
    // nothing here should depend on interrupts ... since we might have been servicing an interrupt
    while(true) {
        set_led_color(10, 0, 0);
        delay(.25); // time stuff normally uses interrupts, but will work (kinda) if interrupts aren't working
        set_led_color(0, 10, 0);
        delay(.25);
        
        float vdd = measure_vdd();
        if(hardware_get_battery_really_dead(vdd)) {
            set_led_color(10, 0, 0);
            delay(0.001);
            poweroff();
        }
    }
}

void * __dso_handle = nullptr;

}


static volatile bool got_filename = false;
static char filename[128] = {0};
void got_date_string(char const *str) {
    strncpy(filename, str, sizeof(filename));
    strncat(filename, ".txt", sizeof(filename));
    got_filename = true;
}

CircularBuffer<CallbackRecord, 128> main_callbacks;

int main(void) {
    clock_setup();
    hardware_init();
    
    set_led_color(10, 0, 0); // draws current, decreasing the battery voltage slightly, purposely done before measuring
    
    time_init();
    serial_setup();
    
    delay(0.0001); // wait for battery voltage to dip
    {
        float vdd = measure_vdd();
        printf("vdd: %f\n", vdd);
        if(hardware_get_battery_dead(vdd)) {
            poweroff();
        }
    }
    
    set_led_color(0, 0, 1); // stop showing red (red won't be visible at all)
    
    sdcard_init();
    
    gps_setup();
    
    auto sdcard_poll_function = []() {
        printf("hello world!\n");
        printf("waiting for date from gps...\n");
        while(!got_filename) yield();
        printf("got date filename: %s! opening\n", filename);
        sdcard_open(filename);
        
        uint8_t const msg[] = "start of log\r\n";
        sdcard_log(sizeof(msg), msg);
        
        gps_start_logging();
        
        set_led_color(0, 1, 0);
        
        while(true) {
            sdcard_poll();
            
            float vdd = measure_vdd();
            if(hardware_get_battery_really_dead(vdd)) {
                set_led_color(10, 0, 0);
                delay(0.001);
                poweroff();
            }
            
            yield();
        }
    };
    Coroutine<1024> sdcard_poll_coroutine;
    sdcard_poll_coroutine.start(sdcard_poll_function);
    
    while(true) {
        assert(!sdcard_poll_coroutine.run_some());
        
        {
            CallbackRecord x;
            while(main_callbacks.read_one(x)) {
                x.call();
            }
        }
    }

    return 0;
}
