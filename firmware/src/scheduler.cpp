#include <utility>
#include <algorithm>

#include "coroutine.h"
#include "time.h"

#include "scheduler.h"

void yield_delay(double dt) {
    assert(false);
    CoroutineBase * cc = current_coroutine;
    assert(cc);
    auto f = [&]() {
        assert(!cc->run_some());
    };
    Runner<decltype(f)> r(f);
    call_at(time_get_ticks() + dt * time_get_ticks_per_second(), r);
    yield();
}

static const unsigned int TIME_CALLBACKS_MAX_LENGTH = 8;

typedef std::pair<uint64_t, RunnerBase *> TimeCallbackElement;
static TimeCallbackElement time_callbacks[TIME_CALLBACKS_MAX_LENGTH];
static uint32_t time_callbacks_length = 0;

void call_at(uint64_t tick, RunnerBase & call) {
    assert(time_callbacks_length != TIME_CALLBACKS_MAX_LENGTH);
    
    time_callbacks[time_callbacks_length].first = tick;
    time_callbacks[time_callbacks_length].second = &call;
    
    time_callbacks_length++;
    
    std::push_heap(time_callbacks, time_callbacks+time_callbacks_length,
        [](TimeCallbackElement const & a, TimeCallbackElement const & b) {
            return a.first > b.first;
        });
}


void scheduler_tick() {
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
