#include "reactor.h"

#include "time.h"

void delay2(double dt) {
    CoroutineBase * cc = current_coroutine;
    auto f = [&]() {
        assert(!cc->run_some());
    };
    Runner<decltype(f)> r(f);
    call_at(time_get_ticks() + dt * time_get_ticks_per_second(), r);
    yield();
}

CircularBuffer<CallbackRecord, 128> main_callbacks;

typedef std::pair<uint64_t, RunnerBase *> TimeCallbackElement;
static TimeCallbackElement time_callbacks[128];
static uint32_t time_callbacks_length = 0;

void call_at(uint64_t tick, RunnerBase & call) {
    assert(time_callbacks_length != 128);
    
    time_callbacks[time_callbacks_length].first = tick;
    time_callbacks[time_callbacks_length].second = &call;
    
    time_callbacks_length++;
    
    std::push_heap(time_callbacks, time_callbacks+time_callbacks_length,
        [](TimeCallbackElement const & a, TimeCallbackElement const & b) {
            return a.first > b.first;
        });
}

void reactor_run() {
    while(true) {
        {
            CallbackRecord x;
            while(main_callbacks.read_one(x)) {
                x.call();
            }
        }
        
        {
            uint64_t time = time_get_ticks();
            
            while(time_callbacks_length && time >= time_callbacks[0].first) {
                time_callbacks[0].second->run();
                std::pop_heap(time_callbacks, time_callbacks+time_callbacks_length,
                    [](TimeCallbackElement const & a, TimeCallbackElement const & b) {
                        return a.first > b.first;
                    });
                time_callbacks_length--;
            }
        }
    }
}
