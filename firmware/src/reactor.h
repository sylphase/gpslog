#ifndef GUARD_FWNFCWEBEIVXGKRY
#define GUARD_FWNFCWEBEIVXGKRY

#include "circular_buffer.h"
#include "coroutine.h"
#include "lock.h"

typedef void (*CallbackFuncType)(void *, uint32_t);
class CallbackRecord {
    CallbackFuncType func_;
    void * arg1_;
    uint32_t arg2_;
public:
    CallbackRecord() : func_(nullptr), arg1_(nullptr), arg2_(0) {
    }
    CallbackRecord(CallbackFuncType func, void * arg1, uint32_t arg2) :
        func_(func), arg1_(arg1), arg2_(arg2) {
        assert(func);
    }
    void call() {
        assert(func_);
        func_(arg1_, arg2_);
    }
};
extern CircularBuffer<CallbackRecord, 128> main_callbacks;

void call_at(uint64_t tick, RunnerBase & call);

void yield_delay(double dt);

void reactor_run();

extern Lock stdlib_lock;

template<typename... Args>
int my_printf(Args... args) {
    Lock::User lu(stdlib_lock);
    return printf(args...);
}

#endif
