#ifndef GUARD_OGAPHGVYGDTBGDYV
#define GUARD_OGAPHGVYGDTBGDYV

#include <cassert>

#include <libopencm3/cm3/cortex.h>

#include "lock.h"

extern Lock stdlib_lock;

template<typename... Args>
int my_printf(Args... args) {
    Lock::User lu(stdlib_lock);
    return printf(args...);
}

class CriticalSection {
    bool masked_prior_;
public:
    CriticalSection() :
        masked_prior_(cm_is_masked_interrupts()) {
        if(!masked_prior_) {
            cm_disable_interrupts();
        }
    }
    ~CriticalSection() {
        assert(cm_is_masked_interrupts());
        if(!masked_prior_) {
            cm_enable_interrupts();
        }
    }
};

#endif