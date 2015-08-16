#include <cstdio>
#include <cassert>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>

#include "sdcard.h"
#include "time.h"
#include "ff/ff.h"
#include "ff/diskio.h"
#include "coroutine.h"
#include "reactor.h"

/*
        PA15=sd_spi_nCS, # SPI1_NSS
        PB3=sd_spi.SCLK, # SPI1_SCK
        PB4=sd_spi.MISO, # SPI1_MISO
        PB5=sd_spi.MOSI, # SPI1_MOSI
*/

static inline uint8_t CRC7_update(uint8_t state, uint8_t byte) {
    static const uint8_t CRC7[256] = {0, 9, 18, 27, 36, 45, 54, 63, 72, 65, 90, 83, 108, 101, 126, 119, 25, 16, 11, 2, 61, 52, 47, 38, 81, 88, 67, 74, 117, 124, 103, 110, 50, 59, 32, 41, 22, 31, 4, 13, 122, 115, 104, 97, 94, 87, 76, 69, 43, 34, 57, 48, 15, 6, 29, 20, 99, 106, 113, 120, 71, 78, 85, 92, 100, 109, 118, 127, 64, 73, 82, 91, 44, 37, 62, 55, 8, 1, 26, 19, 125, 116, 111, 102, 89, 80, 75, 66, 53, 60, 39, 46, 17, 24, 3, 10, 86, 95, 68, 77, 114, 123, 96, 105, 30, 23, 12, 5, 58, 51, 40, 33, 79, 70, 93, 84, 107, 98, 121, 112, 7, 14, 21, 28, 35, 42, 49, 56, 65, 72, 83, 90, 101, 108, 119, 126, 9, 0, 27, 18, 45, 36, 63, 54, 88, 81, 74, 67, 124, 117, 110, 103, 16, 25, 2, 11, 52, 61, 38, 47, 115, 122, 97, 104, 87, 94, 69, 76, 59, 50, 41, 32, 31, 22, 13, 4, 106, 99, 120, 113, 78, 71, 92, 85, 34, 43, 48, 57, 6, 15, 20, 29, 37, 44, 55, 62, 1, 8, 19, 26, 109, 100, 127, 118, 73, 64, 91, 82, 60, 53, 46, 39, 24, 17, 10, 3, 116, 125, 102, 111, 80, 89, 66, 75, 23, 30, 5, 12, 51, 58, 33, 40, 95, 86, 77, 68, 123, 114, 105, 96, 14, 7, 28, 21, 42, 35, 56, 49, 70, 79, 84, 93, 98, 107, 112, 121};
    return CRC7[(state << 1) ^ byte];
}

static inline uint16_t CRC16_update(uint16_t state, uint8_t byte) {
    static uint16_t CRC16[256] = {0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 45419, 49548, 53677, 57806, 61935, 4657, 528, 12915, 8786, 21173, 17044, 29431, 25302, 37689, 33560, 45947, 41818, 54205, 50076, 62463, 58334, 9314, 13379, 1056, 5121, 25830, 29895, 17572, 21637, 42346, 46411, 34088, 38153, 58862, 62927, 50604, 54669, 13907, 9842, 5649, 1584, 30423, 26358, 22165, 18100, 46939, 42874, 38681, 34616, 63455, 59390, 55197, 51132, 18628, 22757, 26758, 30887, 2112, 6241, 10242, 14371, 51660, 55789, 59790, 63919, 35144, 39273, 43274, 47403, 23285, 19156, 31415, 27286, 6769, 2640, 14899, 10770, 56317, 52188, 64447, 60318, 39801, 35672, 47931, 43802, 27814, 31879, 19684, 23749, 11298, 15363, 3168, 7233, 60846, 64911, 52716, 56781, 44330, 48395, 36200, 40265, 32407, 28342, 24277, 20212, 15891, 11826, 7761, 3696, 65439, 61374, 57309, 53244, 48923, 44858, 40793, 36728, 37256, 33193, 45514, 41451, 53516, 49453, 61774, 57711, 4224, 161, 12482, 8419, 20484, 16421, 28742, 24679, 33721, 37784, 41979, 46042, 49981, 54044, 58239, 62302, 689, 4752, 8947, 13010, 16949, 21012, 25207, 29270, 46570, 42443, 38312, 34185, 62830, 58703, 54572, 50445, 13538, 9411, 5280, 1153, 29798, 25671, 21540, 17413, 42971, 47098, 34713, 38840, 59231, 63358, 50973, 55100, 9939, 14066, 1681, 5808, 26199, 30326, 17941, 22068, 55628, 51565, 63758, 59695, 39368, 35305, 47498, 43435, 22596, 18533, 30726, 26663, 6336, 2273, 14466, 10403, 52093, 56156, 60223, 64286, 35833, 39896, 43963, 48026, 19061, 23124, 27191, 31254, 2801, 6864, 10931, 14994, 64814, 60687, 56684, 52557, 48554, 44427, 40424, 36297, 31782, 27655, 23652, 19525, 15522, 11395, 7392, 3265, 61215, 65342, 53085, 57212, 44955, 49082, 36825, 40952, 28183, 32310, 20053, 24180, 11923, 16050, 3793, 7920};
    return CRC16[(state >> 8) ^ byte] ^ ((state & 255) << 8);
}

enum class CMDFormat {
    R1,
    R1b,
    R2,
    R3,
    R7,
};
enum class CMDData {
    NONE,
    READ,
    WRITE,
};

static CoroutineBase *coroutine_waiting_for_spi1_interrupt = nullptr;

static void got_byte(void *, uint32_t) {
    assert(coroutine_waiting_for_spi1_interrupt);
    CoroutineBase *x = coroutine_waiting_for_spi1_interrupt;
    coroutine_waiting_for_spi1_interrupt = nullptr;
    assert(!x->run_some());
}

extern "C" {

void spi1_isr(void) {
    assert(SPI_CR2(SPI1) & SPI_CR2_RXNEIE);
    assert(SPI_SR(SPI1) & SPI_SR_RXNE);
    
    SPI_DR(SPI1); // clear interrupt
    
    assert(main_callbacks.write_one(CallbackRecord(got_byte, nullptr, 0)));
}

}

uint8_t my_spi_xfer(uint8_t data) {
    SPI_DR(SPI1) = data;
    assert(!coroutine_waiting_for_spi1_interrupt);
    coroutine_waiting_for_spi1_interrupt = current_coroutine;
    yield();
    return SPI_DR(SPI1);
}

static uint16_t CMD(uint8_t n, uint32_t argument,
CMDFormat format=CMDFormat::R1, uint32_t * R2_R7_data=nullptr,
CMDData data_mode=CMDData::NONE, uint16_t bytes=0, uint8_t *data=nullptr) {
    assert(n <= 63);
    uint8_t crc = 0;
#define SEND(b) { \
    uint8_t _ = (b); \
    my_spi_xfer(_); \
    crc = CRC7_update(crc, _); \
}
    gpio_clear(GPIOA, GPIO15);
    SEND(0b01000000 | n);
    SEND((argument >> 24) & 255);
    SEND((argument >> 16) & 255);
    SEND((argument >>  8) & 255);
    SEND((argument >>  0) & 255);
    my_spi_xfer((crc << 1) | 1);
    uint16_t resp;
    while(true) { // should abort after 8 bytes (N_CR)
        resp = my_spi_xfer(0xFF);
        if((resp & 0b10000000) == 0) {
            break;
        }
    }
    if(format == CMDFormat::R2) {
        resp = (resp << 8) | my_spi_xfer(0xFF);
    }
    if(format == CMDFormat::R1b) {
        while(my_spi_xfer(0xFF) == 0);
    }
    if(format == CMDFormat::R3 || format == CMDFormat::R7) {
        *R2_R7_data = 0;
        for(int i = 0; i < 4; i++) {
            *R2_R7_data <<= 8;
            *R2_R7_data |= my_spi_xfer(0xFF);
        }
    }
    if(data_mode == CMDData::READ) {
        uint8_t token;
        while(true) {
            token = my_spi_xfer(0xFF);
            if(token != 0xFF) break;
        }
        assert(token == 0xFE);
        uint16_t data_crc = 0;
        for(uint16_t i = 0; i < bytes; i++) {
            uint8_t byte = my_spi_xfer(0xFF);
            *data++ = byte;
            data_crc = CRC16_update(data_crc, byte);
        }
        data_crc = CRC16_update(data_crc, my_spi_xfer(0xFF)); // CRC
        data_crc = CRC16_update(data_crc, my_spi_xfer(0xFF));
        assert(data_crc == 0);
    } else if(data_mode == CMDData::WRITE) {
        my_spi_xfer(0xFF);
        my_spi_xfer(0xFE);
        uint16_t data_crc = 0;
        for(uint16_t i = 0; i < bytes; i++) {
            uint8_t byte = *data++;
            data_crc = CRC16_update(data_crc, byte);
            my_spi_xfer(byte);
        }
        my_spi_xfer(data_crc >> 8); // CRC
        my_spi_xfer(data_crc & 0xFF);
        uint8_t data_resp = my_spi_xfer(0xFF);
        //my_printf("data_resp: %i\n", data_resp);
        assert((data_resp & 0b11111) == 0b00101);
        while(my_spi_xfer(0xFF) != 0xFF);
        //my_printf("done\n");
    }
    my_spi_xfer(0xFF);
    gpio_set(GPIOA, GPIO15);
    yield_delay(1e-6);
    return resp;
}

static bool byte_addresses;
static FATFS fs;
static FIL file;
CircularBuffer<uint8_t, 4096> sdcard_buf;
static uint32_t const SYNC_PERIOD = 10;
static uint64_t next_sync_time;
static bool opened = false;

void sdcard_init() {
    my_printf("hello world from sdcard!\n");
    
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI1_REMAP);
    
    rcc_periph_reset_pulse(RST_SPI1); rcc_periph_clock_enable(RCC_SPI1);
    
    gpio_set(GPIOA, GPIO15);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
    
    //gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO3);
    gpio_set(GPIOB, GPIO4); // pullup
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);
    
    spi_reset(SPI1);
    assert(rcc_apb1_frequency / 256 >= 100e3 && rcc_apb1_frequency / 256 <= 400e3);
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    
    spi_enable(SPI1);
    
    nvic_enable_irq(NVIC_SPI1_IRQ);
    spi_enable_rx_buffer_not_empty_interrupt(SPI1);
    
    // implemented referencing http://elm-chan.org/docs/mmc/mmc_e.html
    
    yield_delay(1e-3);
    
    for(int i = 0; i < (74+7)/8; i++) my_spi_xfer(0xFF);
    
    assert(CMD(0, 0) == 1); // reset, in idle state now
    
    assert(CMD(59, 1) == 1); // enable CRC checking
    
    // XXX code after here only supports SD Ver.2
    uint32_t CMD8_aux; assert(CMD(8, 0x1AA, CMDFormat::R7, &CMD8_aux) == 1); // check voltage
    assert(CMD8_aux == 0x1AA);
    
    while(true) {
        assert(CMD(55, 0) == 1); // ACMD prefix
        
        uint8_t resp = CMD(41, 0x40000000); // ACMD41(0x40000000)
        assert(resp == 0 || resp == 1);
        if(resp == 0) break;
    }
    
    uint8_t csd_bytes[16];
    assert(CMD(9, 0, CMDFormat::R1, nullptr, CMDData::READ, sizeof(csd_bytes), csd_bytes) == 0);
    
    assert(rcc_apb1_frequency / 4 <= 25e6);
    spi_set_baudrate_prescaler(SPI1, 1);
    
    uint32_t CMD58_aux; assert(CMD(58, 0, CMDFormat::R3, &CMD58_aux) == 0); // read ocr
    
    my_printf("CMD58_aux: %lu\n", CMD58_aux);
    
    if(CMD58_aux & (1<<30)) {
        byte_addresses = false;
    } else {
        byte_addresses = true;
        
        assert(CMD(16, 0x200) == 0); // set block size to 512 bytes
    }
    
    my_printf("byte_addresses: %s\n", byte_addresses ? "true" : "false");
    assert(!byte_addresses); // XXX
    
    //uint8_t write_data[512] = "hello world!\n";
    //assert(CMD(24, 0, CMDFormat::R1, nullptr, CMDData::WRITE, sizeof(write_data), write_data) == 0);
    
    /*for(uint32_t i = 0; ; i++) {
        uint8_t read_data[512];
        assert(CMD(17, i, CMDFormat::R1, nullptr, CMDData::READ, sizeof(read_data), read_data) == 0);
        my_printf("%i\n", i);
    }*/
    
    assert(f_mount(&fs, "", 1) == FR_OK);
    
    my_printf("done\n");
}

void sdcard_open(char const * filename) {
    FRESULT x = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    my_printf("f_open result: %i\n", x);
    assert(x == FR_OK);
    next_sync_time = 0;
    opened = true;
}

void sdcard_poll() {
    if(!opened) return;
    
    uint32_t count = sdcard_buf.read_contiguous_available();
    uint8_t const * data = sdcard_buf.read_pointer();
    
    if(count) {
        UINT written;
        assert(f_write(&file, data, count, &written) == FR_OK);
        assert(written <= count);
        sdcard_buf.read_skip(written);
    }
    
    if(time_get_ticks() >= next_sync_time) {
        f_sync(&file);
        
        next_sync_time = time_get_ticks() + SYNC_PERIOD * time_get_ticks_per_second();
    }
}

void sdcard_log(uint32_t length, uint8_t const * data) {
    if(sdcard_buf.write_available() < length) {
        // drop
        my_printf("sdcard_log dropped something\n");
    } else {
        while(length--) {
            assert(sdcard_buf.write_one(*data++));
        }
    }
}

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
    assert(pdrv == 0);
    
    my_printf("writing to %u sectors starting at %lu\n", count, sector);
    
    while(count--) {
        assert(CMD(24, sector, CMDFormat::R1, nullptr, CMDData::WRITE, 512, const_cast<BYTE*>(buff)) == 0);
        sector++;
        buff += 512;
    }
    
	return RES_OK;
}

static DSTATUS status = STA_NOINIT;

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
    assert(pdrv == 0);
    
    my_printf("reading from %u sectors starting at %lu\n", count, sector);
    
    while(count--) {
        assert(CMD(17, sector, CMDFormat::R1, nullptr, CMDData::READ, 512, buff) == 0);
        sector++;
        buff += 512;
    }
    
	return RES_OK;
}

DSTATUS disk_status (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
    assert(pdrv == 0);
    
    return status;
}

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
    assert(pdrv == 0);
    
    status = 0;
    
    return status;
}

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    assert(pdrv == 0);
    
    if(cmd == CTRL_SYNC) {
        return RES_OK;
    } else if(cmd == GET_SECTOR_COUNT) {
        assert(false);
    } else if(cmd == GET_SECTOR_SIZE) {
        assert(false);
    } else if(cmd == GET_BLOCK_SIZE) {
        assert(false);
    } else {
        my_printf("disk_ioctl got invalid cmd %i\n", cmd);
        return RES_PARERR;
    }
}
#endif
