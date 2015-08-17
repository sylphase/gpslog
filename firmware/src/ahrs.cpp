#include <cstdio>
#include <cassert>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "ahrs.h"
#include "coroutine.h"
#include "time.h"
#include "reactor.h"


static CoroutineBase *coroutine_waiting_for_i2c2_interrupt = nullptr;
static void got_interrupt(void *, uint32_t) {
    assert(coroutine_waiting_for_i2c2_interrupt);
    CoroutineBase *x = coroutine_waiting_for_i2c2_interrupt;
    coroutine_waiting_for_i2c2_interrupt = nullptr;
    assert(!x->run_some());
}
extern "C" {
void i2c2_ev_isr(void) {
    i2c_disable_interrupt(I2C2, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    assert(main_callbacks.write_one(CallbackRecord(got_interrupt, nullptr, 0)));
}
void i2c2_er_isr(void) {
    i2c_disable_interrupt(I2C2, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    assert(main_callbacks.write_one(CallbackRecord(got_interrupt, nullptr, 0)));
}
}
static void yield_interrupt(bool buffer=false, bool check=true) {
    if(check) {
        if(I2C_SR1(I2C2) & (I2C_SR1_SB | I2C_SR1_ADDR | I2C_SR1_ADD10 | I2C_SR1_STOPF | I2C_SR1_BTF)) return;
        if(buffer && (I2C_SR1(I2C2) & (I2C_SR1_RxNE | I2C_SR1_TxE))) return;
        if(I2C_SR1(I2C2) & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT)) return;
    }
    assert(!coroutine_waiting_for_i2c2_interrupt);
    coroutine_waiting_for_i2c2_interrupt = current_coroutine;
    i2c_enable_interrupt(I2C2, (buffer ? I2C_CR2_ITBUFEN : 0) | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    yield();
}


/*
PB2=ahrs_int,
PB10=ahrs_i2c.SCL, # I2C2_SCL
PB11=ahrs_i2c.SDA, # I2C2_SDA
*/

static uint8_t const ADDRESS = 0b0101000;

void i2c_write(uint8_t device_address, uint8_t register_address, uint8_t data) {
    /* Send START condition. */
    i2c_send_start(I2C2);

    /* Waiting for START is send and switched to master mode. */
    yield_interrupt();
    assert((I2C_SR1(I2C2) & I2C_SR1_SB)
        && (I2C_SR2(I2C2) & (I2C_SR2_MSL | I2C_SR2_BUSY)));

    /* Send destination address. */
    i2c_send_7bit_address(I2C2, device_address, I2C_WRITE);

    /* Waiting for address to be transferred. */
    yield_interrupt();
    assert(I2C_SR1(I2C2) & I2C_SR1_ADDR);

    /* Clearing ADDR condition sequence. */
    I2C_SR2(I2C2);

    /* Sending the data. */
    i2c_send_data(I2C2, register_address);
    yield_interrupt();
    assert(I2C_SR1(I2C2) & I2C_SR1_BTF); /* Await ByteTransferedFlag. */
    i2c_send_data(I2C2, data);
    yield_interrupt();
    assert(I2C_SR1(I2C2) & (I2C_SR1_BTF | I2C_SR1_TxE));

    /* Send STOP condition. */
    i2c_send_stop(I2C2);
}

void i2c_read(uint8_t device_address, uint8_t register_address, uint8_t * data, uint32_t length) {
    /* Send START condition. */
    i2c_send_start(I2C2);

    /* Waiting for START is send and switched to master mode. */
    yield_interrupt();
    assert((I2C_SR1(I2C2) & I2C_SR1_SB)
        && (I2C_SR2(I2C2) & (I2C_SR2_MSL | I2C_SR2_BUSY)));

    /* Say to what address we want to talk to. */
    /* Yes, WRITE is correct - for selecting register in STTS75. */
    i2c_send_7bit_address(I2C2, device_address, I2C_WRITE);

    /* Waiting for address to be transferred. */
    yield_interrupt();
    assert(!(I2C_SR1(I2C2) & I2C_SR1_AF));
    assert(I2C_SR1(I2C2) & I2C_SR1_ADDR);

    /* Clearing ADDR condition sequence. */
    I2C_SR2(I2C2);

    i2c_send_data(I2C2, register_address);
    yield_interrupt();
    assert(I2C_SR1(I2C2) & (I2C_SR1_BTF | I2C_SR1_TxE));

    /*
     * Now we transferred that we want to ACCESS the temperature register.
     * Now we send another START condition (repeated START) and then
     * transfer the destination but with flag READ.
     */
     
    /* Send START condition. */
    i2c_send_start(I2C2);

    /* Waiting for START is send and switched to master mode. */
    do {
        // BTF is still set, but we're waiting for SB... so poll ):
        yield_interrupt(false, false);
    } while(!(I2C_SR1(I2C2) & I2C_SR1_SB));
    assert((I2C_SR1(I2C2) & I2C_SR1_SB)
        && (I2C_SR2(I2C2) & (I2C_SR2_MSL | I2C_SR2_BUSY)));

    /* Say to what address we want to talk to. */
    i2c_send_7bit_address(I2C2, device_address, I2C_READ); 
    
    if(length >= 3) {
        yield_interrupt();
        assert(I2C_SR1(I2C2) & I2C_SR1_ADDR);
        
        I2C_CR1(I2C2) |= I2C_CR1_ACK;
        
        I2C_SR2(I2C2); // clear ADDR
        
        while(length > 3) {
            yield_interrupt(true);
            assert(I2C_SR1(I2C2) & I2C_SR1_RxNE);
            *data++ = I2C_DR(I2C2);
            length--;
        }
        
        yield_interrupt(true);
        assert(I2C_SR1(I2C2) & I2C_SR1_RxNE);
        
        yield_interrupt();
        assert(I2C_SR1(I2C2) & I2C_SR1_BTF);
        I2C_CR1(I2C2) &= ~I2C_CR1_ACK;
        *data++ = I2C_DR(I2C2);
        I2C_CR1(I2C2) |= I2C_CR1_STOP;
        *data++ = I2C_DR(I2C2);
        yield_interrupt(true);
        assert(I2C_SR1(I2C2) & I2C_SR1_RxNE);
        *data++ = I2C_DR(I2C2);
    } else if(length == 2) {
        /* 2-byte receive is a special case. See datasheet POS bit. */
        I2C_CR1(I2C2) |= (I2C_CR1_POS | I2C_CR1_ACK);

        /* Waiting for address to be transferred. */
        yield_interrupt();
        assert(I2C_SR1(I2C2) & I2C_SR1_ADDR);

        /* Clearing ADDR condition sequence. */
        I2C_SR2(I2C2);

        /* Clearing I2C_SR1_ACK. */
        I2C_CR1(I2C2) &= ~I2C_CR1_ACK;

        /* Now the slave should begin to send us the first byte. Await BTF. */
        yield_interrupt();
        assert(I2C_SR1(I2C2) & I2C_SR1_BTF);
        *data++ = I2C_DR(I2C2);

        /*
         * Yes they mean it: we have to generate the STOP condition before
         * saving the 1st byte.
         */
        I2C_CR1(I2C2) |= I2C_CR1_STOP;

        *data++ = I2C_DR(I2C2);

        /* Original state. */
        I2C_CR1(I2C2) &= ~I2C_CR1_POS;
    } else if(length == 1) {
        /* Waiting for address to be transferred. */
        yield_interrupt();
        assert(I2C_SR1(I2C2) & I2C_SR1_ADDR);

        /* Clearing I2C_SR1_ACK. */
        I2C_CR1(I2C2) &= ~I2C_CR1_ACK;

        /* Clearing ADDR condition sequence. */
        I2C_SR2(I2C2);

        I2C_CR1(I2C2) |= I2C_CR1_STOP;
        
        yield_interrupt(true);
        assert(I2C_SR1(I2C2) & I2C_SR1_RxNE);
        
        *data++ = I2C_DR(I2C2);
    } else { // length == 0
        assert(false);
    }
}

static void ahrs_main() {
    yield_delay(1);
    printf("ahrs starting\n");
    
    /* Enable clocks for I2C2 and AFIO. */
    rcc_periph_reset_pulse(RST_I2C2);
    rcc_periph_clock_enable(RCC_I2C2);
    rcc_periph_clock_enable(RCC_AFIO);

    /* Set alternate functions for the SCL and SDA pins of I2C2. */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
              GPIO_I2C2_SCL | GPIO_I2C2_SDA);

    /* Disable the I2C before changing any configuration. */
    i2c_peripheral_disable(I2C2);

    /* APB1 is running at 36MHz. */
    i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);

    /* 400KHz - I2C Fast Mode */
    i2c_set_fast_mode(I2C2);

    /*
     * fclock for I2C is 36MHz APB1 -> cycle time 28ns, low time at 400kHz
     * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
     * Datasheet suggests 0x1e.
     */
    i2c_set_ccr(I2C2, 0x1e);

    /*
     * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
     * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
     * Incremented by 1 -> 11.
     */
    i2c_set_trise(I2C2, 0x0b);

    /* If everything is configured -> enable the peripheral. */
    i2c_peripheral_enable(I2C2);
    
    nvic_enable_irq(NVIC_I2C2_EV_IRQ);
    nvic_enable_irq(NVIC_I2C2_ER_IRQ);
    
    for(unsigned int j = 1; j < 10; j++) {
        uint8_t buf[16];
        printf("ahrs %i =", j);
        for(unsigned int i = 0; i < sizeof(buf); i += j) {
            i2c_read(ADDRESS, i, buf + i, std::min(sizeof(buf) - i, j));
        }
        for(unsigned int i = 0; i < sizeof(buf); i += 1) {
            printf(" %i", buf[i]);
            buf[i] = 0xff;
        }
        printf("\n");
    }
    
    while(true) {
        yield_delay(1);
    }
}

static Coroutine<2048> ahrs_coroutine;
void ahrs_init() {
    ahrs_coroutine.start(ahrs_main);
}
