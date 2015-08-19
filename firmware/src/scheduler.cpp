#include <utility>
#include <algorithm>
#include <cstdio>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include "coroutine.h"
#include "time.h"
#include "misc.h"
#include "reactor.h"

#include "scheduler.h"

static volatile bool timer_running = false;

static void set_timer(uint64_t ticks) {
    assert(ticks);
    uint64_t new_ticks = ticks;
    uint32_t pre = 1;
    while(new_ticks > (1<<16) && pre != (1<<16)) {
        new_ticks >>= 1;
        pre <<= 1;
    }
    
    { CriticalSection cs;
        timer_set_prescaler(TIM2, pre-1);
        timer_set_period(TIM2, 0xffff);
        timer_set_oc_value(TIM2, TIM_OC1, std::min(1ULL<<16, new_ticks)-1);
        TIM2_EGR |= TIM_EGR_UG;
        
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        timer_enable_irq(TIM2, TIM_DIER_CC1IE);
        nvic_clear_pending_irq(NVIC_TIM2_IRQ);
        
        timer_enable_counter(TIM2);
        
        timer_running = true;
    }
}

static void stop_timer() {
    assert(timer_running);
    timer_disable_irq(TIM2, TIM_DIER_CC1IE);
    timer_running = false;
}



static const unsigned int TIME_CALLBACKS_MAX_LENGTH = 8;

typedef std::pair<uint64_t, RunnerBase *> TimeCallbackElement;
static TimeCallbackElement time_callbacks[TIME_CALLBACKS_MAX_LENGTH];
static uint32_t time_callbacks_length = 0;


static void scheduler_tick(bool run_things) {
    uint64_t time = time_get_ticks();
    
    //fprintf(stderr, "scheduler_tick called at %f\n", static_cast<double>(time) / time_get_ticks_per_second());
    
    if(run_things) {
        while(time_callbacks_length && time >= time_callbacks[0].first) {
            RunnerBase * x = time_callbacks[0].second;
            std::pop_heap(time_callbacks, time_callbacks+time_callbacks_length,
                [](TimeCallbackElement const & a, TimeCallbackElement const & b) {
                    return a.first > b.first;
                });
            time_callbacks_length--;
            
            x->run();
            
            time = time_get_ticks();
        }
    }
    
    { CriticalSection cs;
        if(timer_running) stop_timer();
        
        if(time_callbacks_length) {
            set_timer(std::max(time+1, time_callbacks[0].first) - time);
        }
    }
}

static void got_interrupt(void *, uint32_t) {
    scheduler_tick(true);
}

extern "C" {

void tim2_isr(void) {
    assert(timer_running);
    timer_disable_irq(TIM2, TIM_DIER_CC1IE);
    timer_running = false;
    assert(main_callbacks.write_one(CallbackRecord(got_interrupt, nullptr, 0)));
}

}

void scheduler_init() {
    { CriticalSection cs;
        rcc_periph_clock_enable(RCC_TIM2);
        timer_reset(TIM2);
        timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
        timer_update_on_overflow(TIM2);
        timer_enable_oc_output(TIM2, TIM_OC1);
        nvic_enable_irq(NVIC_TIM2_IRQ);
        timer_disable_oc_preload(TIM2, TIM_OC1);
        
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        busy_delay(10e-6);
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        nvic_clear_pending_irq(NVIC_TIM2_IRQ);
    }
    
    set_timer(0xffff00);
}


void call_at(uint64_t tick, RunnerBase & call) {
    assert(time_callbacks_length != TIME_CALLBACKS_MAX_LENGTH);
    
    time_callbacks[time_callbacks_length].first = tick;
    time_callbacks[time_callbacks_length].second = &call;
    
    time_callbacks_length++;
    
    std::push_heap(time_callbacks, time_callbacks+time_callbacks_length,
        [](TimeCallbackElement const & a, TimeCallbackElement const & b) {
            return a.first > b.first;
        });
    
    scheduler_tick(false);
}


void yield_delay(double dt) {
    CoroutineBase * cc = current_coroutine;
    assert(cc);
    auto f = [&]() {
        assert(!cc->run_some());
    };
    Runner<decltype(f)> r(f);
    call_at(time_get_ticks() + dt * time_get_ticks_per_second(), r);
    yield();
}
