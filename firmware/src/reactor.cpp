#include <libopencmsis/core_cm3.h>

#include "misc.h"

#include "reactor.h"

CircularBuffer<CallbackRecord, 128> main_callbacks;

void reactor_run() {
    while(true) {
        while(main_callbacks.read_available()) {
            main_callbacks.read_pointer()->call();
            main_callbacks.read_skip(1);
        }
        
        { CriticalSection cs;
            if(!main_callbacks.read_available()) __WFI();
        }
    }
}
