#ifndef GUARD_GNKHMPAEJQSWJPKI
#define GUARD_GNKHMPAEJQSWJPKI

#include "circular_buffer.h"

void serial_setup();

extern CircularBuffer<uint8_t, 128> serial_buf;

#endif
