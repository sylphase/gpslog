#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>

#include "misc.h"

#include "time.h"

static volatile uint64_t rollovers;

void time_init() {
  { CriticalSection cs;
    systick_set_reload(0xffffff);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    rollovers = 0;
    systick_interrupt_enable();
    systick_counter_enable();
  }
}

static bool check_rollover() {
  if(systick_get_countflag()) {
    rollovers++;
    return true;
  }
  return false;
}

extern "C" {

void sys_tick_handler(void) {
  { CriticalSection cs;
    check_rollover();
  }
}

}

uint64_t time_get_ticks() {
  uint64_t result;
  // keep calculating result until it is calculated without a rollover having
  // happened
  { CriticalSection cs;
    do {
      result = (rollovers << 24) | (0xffffff - systick_get_value());
    } while(check_rollover());
  }
  return result;
}
