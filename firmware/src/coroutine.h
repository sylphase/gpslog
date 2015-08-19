#ifndef GUARD_STYBXVWIUPDPSFKJ
#define GUARD_STYBXVWIUPDPSFKJ

#include <cassert>
#include <csetjmp>
#include <cstdint>

// public interface is just the Coroutine class and yield()

class CoroutineBase {
protected:
    jmp_buf j_;
    jmp_buf j2_;
    volatile uint32_t sentinel_;
public:
    CoroutineBase() :
        sentinel_(0xDEADBEEF) {
    }
    friend inline void sanity_check();
    friend inline void yield();
    friend void runner_helper();
    virtual bool run_some() = 0;
    virtual ~CoroutineBase() {
    };
};

extern CoroutineBase *current_coroutine;

inline void sanity_check() {
    assert(current_coroutine);
    uint8_t * sp;
    asm volatile (
        "mov %0, sp\n"
    : "=r"(sp));
    assert(sp > reinterpret_cast<volatile uint8_t *>(&current_coroutine->sentinel_) + 512);
    assert(current_coroutine->sentinel_ == 0xDEADBEEF);
}

inline void yield() {
    sanity_check();
    if(setjmp(current_coroutine->j2_)) {
    } else {
        longjmp(current_coroutine->j_, 1);
    }
}

class RunnerBase {
public:
    virtual void run() const = 0;
    virtual ~RunnerBase() {
    };
};

template<typename Function>
class Runner : public RunnerBase {
    Function const & func_;
public:
    Runner(Function const & func) :
        func_(func) {
    }
    void run() const {
        func_();
    }
    ~Runner() {
    }
};

extern RunnerBase const * to_run;

void runner_helper();

template<unsigned int StackSize>
class Coroutine : public CoroutineBase {
    uint8_t stack_[StackSize+7];
    bool started_;
public:
    Coroutine() :
        started_(false) {
    }
    template<typename Function>
    Coroutine(Function & func) :
        Coroutine() {
        assert(!start(func));
    }
    template<typename Function>
    bool start(Function & func) { // returns finished
        Runner<Function> runner(func);
        to_run = &runner;
        
        assert(current_coroutine != this);
        CoroutineBase * old_current_coroutine = current_coroutine;
        
        assert(!started_);
        int x = setjmp(j_);
        if(x) {
            current_coroutine = old_current_coroutine;
            bool finished = x == 2;
            started_ = !finished;
            return finished;
        } else {
            current_coroutine = this;
            started_ = true;
            uint32_t new_sp = reinterpret_cast<uint32_t>(stack_ + StackSize + 7) & (~7);
            asm volatile (
                "mov sp, %0\n"
                "bx %1\n"
            : : "r" (new_sp), "r" (runner_helper));
            assert(false);
        }
    }
    bool run_some() {
        assert(current_coroutine != this);
        CoroutineBase * old_current_coroutine = current_coroutine;
        
        assert(started_);
        int x = setjmp(j_);
        if(x) {
            current_coroutine = old_current_coroutine;
            bool finished = x == 2;
            started_ = !finished;
            return finished;
        } else {
            current_coroutine = this;
            longjmp(j2_, 1);
        }
    }
    ~Coroutine() {
        assert(!started_);
    }
};

#endif
