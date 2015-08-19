#include <cstdio>
#include <cassert>
#include <cstring>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "coroutine.h"
#include "time.h"
#include "reactor.h"
#include "scheduler.h"
#include "misc.h"
#include "gps.h"

#include "ahrs.h"


static CoroutineBase *coroutine_waiting_for_i2c2_interrupt = nullptr;
static void got_interrupt() {
    assert(coroutine_waiting_for_i2c2_interrupt);
    CoroutineBase *x = coroutine_waiting_for_i2c2_interrupt;
    coroutine_waiting_for_i2c2_interrupt = nullptr;
    assert(!x->run_some());
}
static Runner<decltype(got_interrupt)> got_interrupt_runner(got_interrupt);
extern "C" {
void i2c2_ev_isr(void) {
    i2c_disable_interrupt(I2C2, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    reactor_run_in_main(got_interrupt_runner);
}
void i2c2_er_isr(void) {
    i2c_disable_interrupt(I2C2, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    reactor_run_in_main(got_interrupt_runner);
}
}
static void yield_interrupt(bool buffer=false, bool check=true) {
    if(check) {
        if(I2C_SR1(I2C2) & (I2C_SR1_SB | I2C_SR1_ADDR | I2C_SR1_ADD10 | I2C_SR1_STOPF | I2C_SR1_BTF)) return;
        if(buffer && (I2C_SR1(I2C2) & (I2C_SR1_RxNE | I2C_SR1_TxE))) return;
        if(I2C_SR1(I2C2) & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT)) return;
    }
    while(true) {
        assert(!coroutine_waiting_for_i2c2_interrupt);
        coroutine_waiting_for_i2c2_interrupt = current_coroutine;
        i2c_enable_interrupt(I2C2, (buffer ? I2C_CR2_ITBUFEN : 0) | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
        yield();
        if(I2C_SR1(I2C2) & (I2C_SR1_SB | I2C_SR1_ADDR | I2C_SR1_ADD10 | I2C_SR1_STOPF | I2C_SR1_BTF)) return;
        if(buffer && (I2C_SR1(I2C2) & (I2C_SR1_RxNE | I2C_SR1_TxE))) return;
        if(I2C_SR1(I2C2) & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT)) return;
        //my_printf("spurious ahrs i2c interrupt!\n");
    }
}

void i2c_write(uint8_t device_address, uint8_t register_address, uint8_t data) {
    while(I2C_SR2(I2C2) & I2C_SR2_MSL) yield_delay(1e-6);
    
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
    assert(!(I2C_SR1(I2C2) & I2C_SR1_AF));
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
    while(I2C_SR2(I2C2) & I2C_SR2_MSL) yield_delay(1e-6);
    
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
        assert(!(I2C_SR1(I2C2) & I2C_SR1_AF));
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
        assert(!(I2C_SR1(I2C2) & I2C_SR1_AF));
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
        assert(!(I2C_SR1(I2C2) & I2C_SR1_AF));
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

enum class Register : uint8_t {
    /* Page id register definition */
    PAGE_ID = 0X07,
    /* PAGE0 REGISTER DEFINITION START*/
    CHIP_ID = 0x00,
    ACCEL_REV_ID = 0x01,
    MAG_REV_ID = 0x02,
    GYRO_REV_ID = 0x03,
    SW_REV_ID_LSB = 0x04,
    SW_REV_ID_MSB = 0x05,
    BL_REV_ID = 0X06,
    /* Accel data register */
    ACC_DATA_X_LSB = 0X08,
    ACC_DATA_X_MSB = 0X09,
    ACC_DATA_Y_LSB = 0X0A,
    ACC_DATA_Y_MSB = 0X0B,
    ACC_DATA_Z_LSB = 0X0C,
    ACC_DATA_Z_MSB = 0X0D,
    /* Mag data register */
    MAG_DATA_X_LSB = 0X0E,
    MAG_DATA_X_MSB = 0X0F,
    MAG_DATA_Y_LSB = 0X10,
    MAG_DATA_Y_MSB = 0X11,
    MAG_DATA_Z_LSB = 0X12,
    MAG_DATA_Z_MSB = 0X13,
    /* Gyro data registers */
    GYRO_DATA_X_LSB = 0X14,
    GYRO_DATA_X_MSB = 0X15,
    GYRO_DATA_Y_LSB = 0X16,
    GYRO_DATA_Y_MSB = 0X17,
    GYRO_DATA_Z_LSB = 0X18,
    GYRO_DATA_Z_MSB = 0X19,
    /* Euler data registers */
    EULER_H_LSB = 0X1A,
    EULER_H_MSB = 0X1B,
    EULER_R_LSB = 0X1C,
    EULER_R_MSB = 0X1D,
    EULER_P_LSB = 0X1E,
    EULER_P_MSB = 0X1F,
    /* Quaternion data registers */
    QUATERNION_DATA_W_LSB = 0X20,
    QUATERNION_DATA_W_MSB = 0X21,
    QUATERNION_DATA_X_LSB = 0X22,
    QUATERNION_DATA_X_MSB = 0X23,
    QUATERNION_DATA_Y_LSB = 0X24,
    QUATERNION_DATA_Y_MSB = 0X25,
    QUATERNION_DATA_Z_LSB = 0X26,
    QUATERNION_DATA_Z_MSB = 0X27,
    /* Linear acceleration data registers */
    LINEAR_ACCEL_DATA_X_LSB = 0X28,
    LINEAR_ACCEL_DATA_X_MSB = 0X29,
    LINEAR_ACCEL_DATA_Y_LSB = 0X2A,
    LINEAR_ACCEL_DATA_Y_MSB = 0X2B,
    LINEAR_ACCEL_DATA_Z_LSB = 0X2C,
    LINEAR_ACCEL_DATA_Z_MSB = 0X2D,
    /* Gravity data registers */
    GRAVITY_DATA_X_LSB = 0X2E,
    GRAVITY_DATA_X_MSB = 0X2F,
    GRAVITY_DATA_Y_LSB = 0X30,
    GRAVITY_DATA_Y_MSB = 0X31,
    GRAVITY_DATA_Z_LSB = 0X32,
    GRAVITY_DATA_Z_MSB = 0X33,
    /* Temperature data register */
    TEMP = 0X34,
    /* Status registers */
    CALIB_STAT = 0X35,
    SELFTEST_RESULT = 0X36,
    INTR_STAT = 0X37,
    SYS_CLK_STAT = 0X38,
    SYS_STAT = 0X39,
    SYS_ERR = 0X3A,
    /* Unit selection register */
    UNIT_SEL = 0X3B,
    DATA_SELECT = 0X3C,
    /* Mode registers */
    OPR_MODE = 0X3D,
    PWR_MODE = 0X3E,
    SYS_TRIGGER = 0X3F,
    TEMP_SOURCE = 0X40,
    /* Axis remap registers */
    AXIS_MAP_CONFIG = 0X41,
    AXIS_MAP_SIGN = 0X42,
    /* SIC registers */
    SIC_MATRIX_0_LSB = 0X43,
    SIC_MATRIX_0_MSB = 0X44,
    SIC_MATRIX_1_LSB = 0X45,
    SIC_MATRIX_1_MSB = 0X46,
    SIC_MATRIX_2_LSB = 0X47,
    SIC_MATRIX_2_MSB = 0X48,
    SIC_MATRIX_3_LSB = 0X49,
    SIC_MATRIX_3_MSB = 0X4A,
    SIC_MATRIX_4_LSB = 0X4B,
    SIC_MATRIX_4_MSB = 0X4C,
    SIC_MATRIX_5_LSB = 0X4D,
    SIC_MATRIX_5_MSB = 0X4E,
    SIC_MATRIX_6_LSB = 0X4F,
    SIC_MATRIX_6_MSB = 0X50,
    SIC_MATRIX_7_LSB = 0X51,
    SIC_MATRIX_7_MSB = 0X52,
    SIC_MATRIX_8_LSB = 0X53,
    SIC_MATRIX_8_MSB = 0X54,
    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB = 0X55,
    ACCEL_OFFSET_X_MSB = 0X56,
    ACCEL_OFFSET_Y_LSB = 0X57,
    ACCEL_OFFSET_Y_MSB = 0X58,
    ACCEL_OFFSET_Z_LSB = 0X59,
    ACCEL_OFFSET_Z_MSB = 0X5A,
    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB = 0X5B,
    MAG_OFFSET_X_MSB = 0X5C,
    MAG_OFFSET_Y_LSB = 0X5D,
    MAG_OFFSET_Y_MSB = 0X5E,
    MAG_OFFSET_Z_LSB = 0X5F,
    MAG_OFFSET_Z_MSB = 0X60,
    /* Gyroscope Offset registers */
    GYRO_OFFSET_X_LSB = 0X61,
    GYRO_OFFSET_X_MSB = 0X62,
    GYRO_OFFSET_Y_LSB = 0X63,
    GYRO_OFFSET_Y_MSB = 0X64,
    GYRO_OFFSET_Z_LSB = 0X65,
    GYRO_OFFSET_Z_MSB = 0X66,
    /* Radius registers */
    ACCEL_RADIUS_LSB = 0X67,
    ACCEL_RADIUS_MSB = 0X68,
    MAG_RADIUS_LSB = 0X69,
    MAG_RADIUS_MSB = 0X6A,
};
enum class PowerMode : uint8_t {
    NORMAL = 0X00,
    LOWPOWER = 0X01,
    SUSPEND = 0X02,
};
enum OperationMode : uint8_t {
    CONFIG = 0X00,
    ACCONLY = 0X01,
    MAGONLY = 0X02,
    GYRONLY = 0X03,
    ACCMAG = 0X04,
    ACCGYRO = 0X05,
    MAGGYRO = 0X06,
    AMG = 0X07,
    IMUPLUS = 0X08,
    COMPASS = 0X09,
    M4G = 0X0A,
    NDOF_FMC_OFF = 0X0B,
    NDOF = 0X0C,
};


/*
PB2=ahrs_int,
PB10=ahrs_i2c.SCL, # I2C2_SCL
PB11=ahrs_i2c.SDA, # I2C2_SDA
*/

static uint8_t const ADDRESS = 0b0101000;

template<typename T>
void write(Register r, T v) {
    i2c_write(ADDRESS, static_cast<uint8_t>(r), static_cast<uint8_t>(v));
}
uint8_t read(Register r) {
    uint8_t res;
    i2c_read(ADDRESS, static_cast<uint8_t>(r), &res, 1);
    return res;
}

static void ahrs_main() {
    yield_delay(1);
    my_printf("ahrs starting\n");
    
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
    
    write(Register::PAGE_ID, 0);
    assert(read(Register::CHIP_ID) == 0xA0);
    write(Register::OPR_MODE, OperationMode::CONFIG); yield_delay(30e-3);
    write(Register::SYS_TRIGGER, 0x20); yield_delay(1); // do reset
    
    //write(Register::SYS_TRIGGER, 1); yield_delay(1); // do self test
    
    assert(read(Register::CHIP_ID) == 0xA0);
    assert(read(Register::SYS_STAT) == 0);
    assert(read(Register::SELFTEST_RESULT) == 15);
    
    write(Register::OPR_MODE, OperationMode::NDOF); yield_delay(30e-3);
    
    my_printf("ahrs initialized\n");
    
    while(true) {
        uint8_t constexpr start = static_cast<uint8_t>(Register::ACC_DATA_X_LSB);
        uint8_t constexpr end = static_cast<uint8_t>(Register::CALIB_STAT);
        uint8_t buf[2+(end-start)];
        buf[0] = 0; // custom message type
        buf[1] = 3; // ahrs measurement
        i2c_read(ADDRESS, start, buf+2, end-start);
        gps_write_packet(buf, sizeof(buf)); // might drop
    }
}

static Coroutine<2048> ahrs_coroutine;
void ahrs_init() {
    ahrs_coroutine.start(ahrs_main);
}
