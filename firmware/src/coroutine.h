#ifndef GUARD_STYBXVWIUPDPSFKJ
#define GUARD_STYBXVWIUPDPSFKJ

#include <cassert>
#include <csetjmp>
#include <cstdint>

class CoroutineBase {
protected:
    jmp_buf j_;
    jmp_buf j2_;
public:
    friend inline void yield();
    friend void runner_helper();
};

extern CoroutineBase *current_coroutine;

inline void yield() {
    assert(current_coroutine);
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
    bool started_;
    uint8_t stack_[StackSize+7];
public:
    Coroutine() :
        started_(false) {
    }
    template<typename Function>
    bool start(Function const & func) { // returns finished
        Runner<Function> runner(func);
        to_run = &runner;
        
        assert(!started_);
        int x = setjmp(j_);
        if(x) {
            current_coroutine = nullptr;
            bool finished = x == 2;
            started_ = !finished;
            return finished;
        } else {
            assert(!current_coroutine);
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
        assert(started_);
        int x = setjmp(j_);
        if(x) {
            current_coroutine = nullptr;
            bool finished = x == 2;
            started_ = !finished;
            return finished;
        } else {
            assert(!current_coroutine);
            current_coroutine = this;
            longjmp(j2_, 1);
        }
    }
};

#endif
