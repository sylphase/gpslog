#include <libopencmsis/core_cm3.h>

#include "reactor.h"

CircularBuffer<CallbackRecord, 128> main_callbacks;

void reactor_run() {
    while(true) {
        while(main_callbacks.read_available()) {
            main_callbacks.read_pointer()->call();
            main_callbacks.read_skip(1);
        }
        
        cm_disable_interrupts();
        if(!main_callbacks.read_available()) __WFI();
        cm_enable_interrupts();
    }
}
