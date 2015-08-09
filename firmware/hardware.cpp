#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/cortex.h>

#include "hardware.h"

void set_led_color(LEDColor x) {
    gpio_set(GPIOA, GPIO6);
    gpio_set(GPIOA, GPIO7);
    gpio_set(GPIOB, GPIO0);
    if(x == LEDColor::RED) gpio_clear(GPIOA, GPIO6);
    if(x == LEDColor::GREEN) gpio_clear(GPIOA, GPIO7);
    if(x == LEDColor::BLUE) gpio_clear(GPIOB, GPIO0);
}

static void adc_setup(void) {
    rcc_periph_reset_pulse(RST_ADC1); rcc_periph_clock_enable(RCC_ADC1);
    
    /* Make sure the ADC doesn't run during config. */
    adc_off(ADC1);
    /* We configure everything for one single conversion. */
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
    adc_power_on(ADC1);
    /* Wait for ADC starting up. */
    for (int i = 0; i < 800000; i++) /* Wait a bit. */
        __asm__("nop");
    adc_reset_calibration(ADC1);
    adc_calibration(ADC1);
    adc_enable_temperature_sensor(ADC1);
}

static uint16_t read_adc_naiive(uint8_t channel) {
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_direct(ADC1);
    while (!adc_eoc(ADC1));
    uint16_t reg16 = adc_read_regular(ADC1);
    return reg16;
}

static float sample_reference_averaged() {
    double res = 0;
    for(int i = 0; i < 10; i++) res += read_adc_naiive(17)/4095.;
    return res/10;
}

float measure_vdd() {
    return 1.20/sample_reference_averaged();
}

bool hardware_get_battery_dead() {
    double vdd_inf = 1.24/sample_reference_averaged();
    return vdd_inf < 3.3*.98;
}

bool hardware_get_battery_really_dead() {
    double vdd_sup = 1.16/sample_reference_averaged();
    return vdd_sup < 3;
}

void poweroff() {
    cm_disable_interrupts();
    gpio_set(GPIOB, GPIO1);
    while(true) { }
}

void hardware_init() {
    // status_led
    gpio_set(GPIOA, GPIO6);
    gpio_set(GPIOA, GPIO7);
    gpio_set(GPIOB, GPIO0);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
    
    // vcc3_3_enable_pulldown
    gpio_clear(GPIOB, GPIO1);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
    
    adc_setup();
}
