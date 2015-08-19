#include <libopencmsis/core_cm3.h>

#include "misc.h"

#include "reactor.h"

CircularBuffer<RunnerBase const *, 8> main_callbacks;

void reactor_run() {
    while(true) {
        while(main_callbacks.read_available()) {
            (*main_callbacks.read_pointer())->run();
            main_callbacks.read_skip(1);
        }
        
        { CriticalSection cs;
            if(!main_callbacks.read_available()) __WFI();
        }
    }
}

void reactor_run_in_main(RunnerBase const & x) {
    assert(main_callbacks.write_one(&x));
}
