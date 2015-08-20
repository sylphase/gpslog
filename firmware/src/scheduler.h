#ifndef GUARD_NZHWPWFELBEOBXFI
#define GUARD_NZHWPWFELBEOBXFI

void scheduler_init();

void call_at(uint64_t tick, RunnerBase & call);

void yield_until(uint64_t tick);

void yield_delay(double dt);

#endif
