#include "coroutine.h"

CoroutineBase *current_coroutine = nullptr;
RunnerBase const * to_run = nullptr;

void runner_helper() {
    to_run->run();
    longjmp(current_coroutine->j_, 2);
}
