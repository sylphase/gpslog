#ifndef GUARD_OGAPHGVYGDTBGDYV
#define GUARD_OGAPHGVYGDTBGDYV

#include "lock.h"

extern Lock stdlib_lock;

template<typename... Args>
int my_printf(Args... args) {
    Lock::User lu(stdlib_lock);
    return printf(args...);
}

#endif
