#include <cstdio>
#include <cassert>
#include <cstring>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "coroutine.h"
#include "reactor.h"
#include "scheduler.h"
#include "misc.h"
#include "gps.h"
#include "time.h"

#include "baro.h"

/*
        PB12=external_spi_nCS[0], # SPI2_NSS
        PB13=sensor_spi.SCLK, # SPI2_SCK
        PB14=sensor_spi.MISO, # SPI2_MISO
        PB15=sensor_spi.MOSI, # SPI2_MOSI
        
        PB7=imu_spi_nCS,
        PB8=imu_INT,
        PB9=baro_spi_nCS,
        PA5=external_spi_nCS[1],
        PA2=external_spi_nCS[2],
*/

static void send_command(uint8_t cmd, uint32_t read=0, uint8_t * dest=nullptr) {
    gpio_clear(GPIOB, GPIO9);
    SPI_DR(SPI2) = cmd; yield();
    while(read--) {
        SPI_DR(SPI2) = cmd; yield(); *dest++ = SPI_DR(SPI2);
    }
    gpio_set(GPIOB, GPIO9);
}

static Coroutine<2048> baro_coroutine;

struct Result {
    double temperature; // kelvin
    double pressure; // pascals
};

void decode(uint16_t prom[8], uint32_t D1, uint32_t D2, Result & res) {
    double dT = D2 - prom[5] * pow(2, 8);
    double TEMP = 2000 + dT*prom[6]/pow(2, 23);
    
    double T2, OFF2, SENS2;
    if(TEMP < 2000) {
        T2 = pow(dT, 2) / pow(2, 31);
        OFF2 = 5 * pow(TEMP - 2000, 2) / pow(2, 1);
        SENS2 = 5 * pow(TEMP - 2000, 2) / pow(2, 2);
        if(TEMP < -1500) {
            OFF2 += 7 * pow(TEMP + 1500, 2);
            SENS2 += 11 * pow(TEMP + 1500, 2) / pow(2, 1);
        }
    } else {
        T2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }
    TEMP -= T2;
    
    double OFF = prom[2]*pow(2, 16) + prom[4]*dT/pow(2, 7);
    OFF -= OFF2;
    double SENS = prom[1]*pow(2, 15) + prom[3]*dT/pow(2, 8);
    SENS -= SENS2;
    
    res.temperature = TEMP/100 + 273.15;
    res.pressure = (D1*SENS/pow(2, 21) - OFF)/pow(2, 15);
}

static void baro_main() {
    uint16_t prom[8];
    send_command(0x1E); // Reset
    yield_delay(15e-3);
    for(int i = 0; i < 8; i++) {
        uint8_t buf[2];
        send_command(0xA0 + 2 * i, 2, buf); // PROM Read
        prom[i] = (buf[0] << 8) | buf[1];
        my_printf("prom[%i] = %i\n", i, prom[i]);
    }
    {
        uint8_t buf[2+8*2];
        buf[0] = 0; // custom message type
        buf[1] = 1; // barometer prom
        for(int i = 0; i < 8; i++) {
            buf[2+2*i+0] = prom[i] >> 8;
            buf[2+2*i+1] = prom[i] & 255;
        }
        while(!gps_write_packet(buf, sizeof(buf))) {
            // make sure this is logged
            yield_delay(0.1);
        }
    }
    
    uint64_t measurement_period = time_get_ticks_per_second()/10;
    uint64_t measurement_time = (time_get_ticks()/measurement_period + 20) * measurement_period;
    
    while(true) {
        yield_until(measurement_time);
        
        send_command(0x48); // Convert D1 (OSR=4096)
        yield_delay(9.04e-3);
        uint8_t D1[3]; send_command(0x00, 3, D1); // ADC Read
        send_command(0x58); // Convert D2 (OSR=4096)
        yield_delay(9.04e-3);
        uint8_t D2[3]; send_command(0x00, 3, D2); // ADC Read
        
        if(false) {
            Result res; decode(prom,
                (D1[0] << 16) | (D1[1] << 8) | D1[2],
                (D2[0] << 16) | (D2[1] << 8) | D2[2],
            res);
            
            my_printf("temperature %f pressure %f\n", res.temperature, res.pressure);
        }
        
        uint8_t buf[2+2*3];
        buf[0] = 0; // custom message type
        buf[1] = 2; // barometer measurement
        memcpy(buf+2, D1, 3);
        memcpy(buf+5, D2, 3);
        gps_write_packet(buf, sizeof(buf)); // might drop
        
        measurement_time += measurement_period;
    }
}

void baro_init() {
    rcc_periph_reset_pulse(RST_SPI2); rcc_periph_clock_enable(RCC_SPI2);
    
    gpio_set(GPIOB, GPIO9);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13);
    gpio_set(GPIOB, GPIO14); // pullup
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
    
    spi_reset(SPI2);
    assert(rcc_apb1_frequency / 2 <= 20e6);
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);
    
    spi_enable(SPI2);
    
    nvic_enable_irq(NVIC_SPI2_IRQ);
    spi_enable_rx_buffer_not_empty_interrupt(SPI2);
    
    baro_coroutine.start(baro_main);
}

static void got_byte() {
    assert(!baro_coroutine.run_some());
}
static Runner<decltype(got_byte)> got_byte_runner(got_byte);

extern "C" {

void spi2_isr(void) {
    assert(SPI_CR2(SPI2) & SPI_CR2_RXNEIE);
    assert(SPI_SR(SPI2) & SPI_SR_RXNE);
    
    SPI_DR(SPI2); // clear interrupt
    
    reactor_run_in_main(got_byte_runner);
}

}
