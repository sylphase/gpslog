#ifndef GUARD_TSICRJECPMABLADL
#define GUARD_TSICRJECPMABLADL

#include <libopencm3/stm32/rcc.h>

void time_init();
uint64_t time_get_ticks();

inline void delay(double dt) {
  uint64_t end = time_get_ticks() + dt * rcc_ahb_frequency;
  while(time_get_ticks() < end);
}

#endif
