#include <array>
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
#include "hardware.h"

#include "sensors.h"

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

class Pin {
    uint32_t gpioport_;
    uint16_t gpio_;
public:
    Pin(uint32_t gpioport, uint16_t gpio) : gpioport_(gpioport), gpio_(gpio) { }
    void init() {
        gpio_set(gpioport_, gpio_);
        gpio_set_mode(gpioport_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, gpio_);
    }
    void set() { gpio_set(gpioport_, gpio_); }
    void clear() { gpio_clear(gpioport_, gpio_); }
};

static void send_command(Pin & nCS_pin, uint8_t cmd, uint32_t length=0, uint8_t * buf=nullptr) {
    nCS_pin.clear();
    SPI_DR(SPI2) = cmd; yield();
    while(length--) {
        SPI_DR(SPI2) = *buf; yield(); *buf++ = SPI_DR(SPI2);
    }
    nCS_pin.set();
}

Pin baro_nCS_pin(GPIOB, GPIO9);
std::array<Pin, 4> imu_nCS_pins{
    Pin(GPIOB, GPIO12),
    Pin(GPIOA, GPIO5),
    Pin(GPIOA, GPIO2),
    Pin(GPIOB, GPIO7),
};

static Coroutine<2048> sensors_coroutine;

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

bool const just_pause = true;

void imu_mag_write(Pin & nCS_pin, uint8_t addr, uint8_t value) {
    send_command(nCS_pin, 49, 1, std::array<uint8_t, 1>{0x0C}.begin());
    send_command(nCS_pin, 50, 1, std::array<uint8_t, 1>{addr}.begin());
    send_command(nCS_pin, 51, 1, std::array<uint8_t, 1>{value}.begin());
    send_command(nCS_pin, 52, 1, std::array<uint8_t, 1>{0x80}.begin());
    if(just_pause) {
        yield_delay(50e-3);
    } else {
        while(true) {
            uint8_t buf;
            send_command(nCS_pin, 0x80|52, 1, &buf);
            if(!(buf & 0x80)) break;
        }
    }
}

uint8_t imu_mag_read(Pin & nCS_pin, uint8_t addr) {
    send_command(nCS_pin, 49, 1, std::array<uint8_t, 1>{0x80|0x0C}.begin()); // write ADDR
    send_command(nCS_pin, 50, 1, std::array<uint8_t, 1>{addr}.begin()); // write REG
    send_command(nCS_pin, 51, 1, std::array<uint8_t, 1>{0}.begin()); // write DO
    send_command(nCS_pin, 52, 1, std::array<uint8_t, 1>{0x80}.begin());
    if(just_pause) {
        yield_delay(50e-3);
    } else {
        while(true) {
            uint8_t buf = 0;
            send_command(nCS_pin, 0x80|52, 1, &buf);
            if(!(buf & 0x80)) break;
        }
    }
    {
        uint8_t buf;
        send_command(nCS_pin, 0x80|53, 1, &buf);
        return buf;
    }
}

static void sensors_main() {
    my_printf("start\n");
    for(Pin & pin : imu_nCS_pins) {
        send_command(pin, 107, 1, std::array<uint8_t, 1>{0x80}.begin()); // PWR_MGMT_1.H_RESET = 1
        yield_delay(50e-3);
        send_command(pin, 107, 1, std::array<uint8_t, 1>{0x01}.begin()); // PWR_MGMT_1.CLKSEL = 1
        send_command(pin, 106, 1, std::array<uint8_t, 1>{0x30}.begin()); // USER_CTRL.I2C_MST_EN = 1, USER_CTRL.I2C_IF_DIS = 1
        imu_mag_write(pin, 0x0B, 1); // reset
        yield_delay(50e-3);
        imu_mag_write(pin, 0x0A, 0b10110); // 16 bit output, continuous measurement mode 2
        my_printf("WIA: %i\n", imu_mag_read(pin, 0x00));
        send_command(pin, 37, 1, std::array<uint8_t, 1>{0x80|0x0C}.begin()); // write ADDR
        send_command(pin, 38, 1, std::array<uint8_t, 1>{0x02}.begin()); // write REG
        send_command(pin, 39, 1, std::array<uint8_t, 1>{0x80 | 8}.begin()); // write CTRL
    }
    
    //assert(rcc_apb1_frequency / 2 <= 20e6);
    //spi_set_baudrate_prescaler(SPI1, 0);    
    
    uint16_t prom[8];
    { // baro init
        send_command(baro_nCS_pin, 0x1E); // Reset
        yield_delay(15e-3);
        for(int i = 0; i < 8; i++) {
            uint8_t buf[2];
            send_command(baro_nCS_pin, 0xA0 + 2 * i, 2, buf); // PROM Read
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
            while(!gps_write_stamped_packet(buf, sizeof(buf))) {
                // make sure this is logged
                yield_delay(0.1);
            }
        }
    }
    
    uint64_t measurement_period = time_get_ticks_per_second()/10;
    uint64_t measurement_time = (time_get_ticks()/measurement_period + 20) * measurement_period;
    
    while(true) {
        yield_until(measurement_time);
        
        uint8_t packet[2+2*3+4*(14+8)];
        packet[0] = 0; // custom message type
        packet[1] = 4; // barometer+4xIMU measurement
        
        { // barometer
            send_command(baro_nCS_pin, 0x48); // Convert D1 (OSR=4096)
            yield_delay(9.04e-3);
            uint8_t D1[3]; send_command(baro_nCS_pin, 0x00, 3, D1); // ADC Read
            send_command(baro_nCS_pin, 0x58); // Convert D2 (OSR=4096)
            yield_delay(9.04e-3);
            uint8_t D2[3]; send_command(baro_nCS_pin, 0x00, 3, D2); // ADC Read
            
            if(false) {
                Result res; decode(prom,
                    (D1[0] << 16) | (D1[1] << 8) | D1[2],
                    (D2[0] << 16) | (D2[1] << 8) | D2[2],
                res);
                
                my_printf("temperature %f pressure %f\n", res.temperature, res.pressure);
            }
            
            memcpy(packet+2, D1, 3);
            memcpy(packet+5, D2, 3);
        }
        
        for(uint8_t i = 0; i < imu_nCS_pins.size(); i++) {
            Pin & pin = imu_nCS_pins[i];
            
            uint8_t buf[14+8];
            send_command(pin, 0x80 + 59, sizeof(buf), buf);
            
            if(false) {
                my_printf("imu%i ", i);
                for(uint8_t j = 0; j < sizeof(buf); j++) {
                    my_printf("%02X", buf[j]);
                }
                my_printf("\n");
            }
            
            memcpy(packet + 2 + 2*3 + i * (14 + 8), buf, sizeof(buf));
        }
        
        if(!gps_write_stamped_packet(packet, sizeof(packet))) {
            set_led_color(1, 1, 0);
        }
        
        measurement_time += measurement_period;
    }
}

void sensors_init() {
    rcc_periph_reset_pulse(RST_SPI2); rcc_periph_clock_enable(RCC_SPI2);
    
    baro_nCS_pin.init();
    for(Pin & pin : imu_nCS_pins) pin.init();
    
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13);
    gpio_set(GPIOB, GPIO14); // pullup
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
    
    spi_reset(SPI2);
    assert(rcc_apb1_frequency / 64 <= 1e6);
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);
    
    spi_enable(SPI2);
    
    nvic_enable_irq(NVIC_SPI2_IRQ);
    spi_enable_rx_buffer_not_empty_interrupt(SPI2);
    
    sensors_coroutine.start(sensors_main);
}

static void got_byte() {
    assert(!sensors_coroutine.run_some());
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
