#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "hardware.h"
#include "time.h"
#include "serial.h"
#include "gps.h"

static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    
    /* Enable GPIOA, GPIOB, GPIOC clock. */
    rcc_periph_reset_pulse(RST_GPIOA); rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_reset_pulse(RST_GPIOB); rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_reset_pulse(RST_GPIOC); rcc_periph_clock_enable(RCC_GPIOC);
}


int main(void) {
    clock_setup();
    hardware_init();
    time_init();
    
    set_led_color(LEDColor::RED); // draws current, decreasing the battery voltage slightly, purposely done before measuring
    delay(0.0001); // wait for battery voltage to dip
    if(hardware_get_battery_dead()) {
        poweroff();
    }
    
    set_led_color(LEDColor::GREEN); // stop showing red (red won't be visible at all)
    
    serial_setup();
    gps_setup();
    
    delay(1);
    set_led_color(LEDColor::BLUE);
    
    printf("hello world!\r\n");
    
    while (1) {
        printf("vdd: %f\r\n", measure_vdd());
        delay(0.1);
        
        if(hardware_get_battery_really_dead()) {
            set_led_color(LEDColor::RED);
            delay(0.001);
            poweroff();
        }
    }

    return 0;
}
