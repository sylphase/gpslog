#ifndef GUARD_XVSISFHKRXCEXPYB
#define GUARD_XVSISFHKRXCEXPYB

#include "circular_buffer.h"

void sdcard_init();
void sdcard_open(char const * filename);
void sdcard_poll();
void sdcard_log(uint32_t length, uint8_t const * data);

extern CircularBuffer<uint8_t, 2048> sdcard_buf;

#endif
