#ifndef GUARD_XVSISFHKRXCEXPYB
#define GUARD_XVSISFHKRXCEXPYB

#include "circular_buffer.h"

void sdcard_init();
void sdcard_poll();
void sdcard_log(uint32_t length, uint8_t const * data);

extern CircularBuffer<uint8_t, 4096> sdcard_buf;

#endif
