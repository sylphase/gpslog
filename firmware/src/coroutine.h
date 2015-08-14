#ifndef GUARD_STYBXVWIUPDPSFKJ
#define GUARD_STYBXVWIUPDPSFKJ

#include <setjmp.h>

class CoroutineBase {
protected:
    jmp_buf j_;
    jmp_buf j2_;
public:
    friend inline void yield();
};

CoroutineBase *current_coroutine = nullptr;

inline void yield() {
    assert(current_coroutine);
    if(setjmp(current_coroutine->j2_)) {
        printf("resumed\n");
    } else {
        longjmp(current_coroutine->j_, 1);
    }
}

template<unsigned int StackSize>
class Coroutine : public CoroutineBase {
    bool started_;
    uint8_t stack_[StackSize+7];
public:
    Coroutine() :
        started_(false) {
    }
    template<typename Function, typename... Args>
    bool start(Function func, Args... args) { // returns finished
        assert(!started_);
        int x = setjmp(j_);
        if(x) {
            current_coroutine = nullptr;
            printf("got %i\n", x);
            bool finished = x == 2;
            started_ = !finished;
            return finished;
        } else {
            current_coroutine = this;
            started_ = true;
            uint32_t new_sp = reinterpret_cast<uint32_t>(stack_ + StackSize + 7) & (~7);
            printf("new_sp: %lu\n", new_sp);
            asm volatile ("mov sp, %0" : : "r" (new_sp));
            func(args...);
            longjmp(j_, 2);
        }
    }
    bool run_some() {
        assert(started_);
        int x = setjmp(j_);
        if(x) {
            printf("got %i\n", x);
            bool finished = x == 2;
            started_ = !finished;
            return finished;
        } else {
            printf("restarting\n");
            current_coroutine = this;
            longjmp(j2_, 1);
        }
    }
};

#endif
