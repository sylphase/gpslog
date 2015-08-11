#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/cortex.h>

#include "hardware.h"

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
    static int const COUNT = 10;
    for(int i = 0; i < COUNT; i++) res += read_adc_naiive(17);
    return res*(1./COUNT/4095);
}

float measure_vdd() {
    return 1.20/sample_reference_averaged();
}

bool hardware_get_battery_dead(float vdd) {
    double vdd_inf = 1.24/1.20*vdd;
    return vdd_inf < 3.3*.98;
}

bool hardware_get_battery_really_dead(float vdd) {
    double vdd_sup = 1.16/1.20*vdd;
    return vdd_sup < 3;
}

void poweroff() {
    cm_disable_interrupts();
    gpio_set(GPIOB, GPIO1);
    while(true) { }
}

void set_led_color(double red, double green, double blue) {
    red *= 0.6; // compensate for red LED being brighter
    
    double sum = red + green + blue;
    if(sum > 1) {
        red /= sum;
        green /= sum;
        blue /= sum;
    }
    
    red *= 0.1; green *= 0.1; blue *= 0.1;
    
    timer_set_oc_value(TIM3, TIM_OC1, red*0xffff+.5);
    timer_set_oc_value(TIM3, TIM_OC2, (red+green)*0xffff+.5);
    timer_set_oc_value(TIM3, TIM_OC3, (1-blue)*0xffff+.5);
}

void hardware_init() {
    // vcc3_3_enable_pulldown
    gpio_clear(GPIOB, GPIO1);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
    
    adc_setup();
    
    // status_led
    rcc_periph_reset_pulse(RST_TIM3); rcc_periph_clock_enable(RCC_TIM3);
    timer_reset(TIM3);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, 0);
    timer_set_period(TIM3, 0xfffe);

    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_set_oc_polarity_low(TIM3, TIM_OC1);
    timer_disable_oc_preload(TIM3, TIM_OC1);

    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC2);
    timer_set_oc_polarity_low(TIM3, TIM_OC2);
    timer_disable_oc_preload(TIM3, TIM_OC2);

    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC3);
    timer_set_oc_polarity_high(TIM3, TIM_OC3);
    timer_disable_oc_preload(TIM3, TIM_OC3);

    timer_update_on_overflow(TIM3);
    
    set_led_color(0, 0, 0);
    
    TIM3_EGR |= TIM_EGR_UG;
    timer_enable_counter(TIM3);
    
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);
}
