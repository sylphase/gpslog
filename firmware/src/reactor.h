#ifndef GUARD_FWNFCWEBEIVXGKRY
#define GUARD_FWNFCWEBEIVXGKRY

#include "circular_buffer.h"
#include "runner.h"

extern CircularBuffer<RunnerBase const *, 8> main_callbacks;

void reactor_run();

void reactor_run_in_main(RunnerBase const & x);

#endif
