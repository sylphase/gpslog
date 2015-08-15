#ifndef GUARD_TSICRJECPMABLADL
#define GUARD_TSICRJECPMABLADL

#include <libopencm3/stm32/rcc.h>

void time_init();
uint64_t time_get_ticks();

inline uint64_t time_get_ticks_per_second() {
  return rcc_ahb_frequency;
}

inline void busy_delay(double dt) {
  uint64_t end = time_get_ticks() + dt * time_get_ticks_per_second();
  while(time_get_ticks() < end);
}

#endif
