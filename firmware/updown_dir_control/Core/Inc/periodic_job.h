#ifndef PERIODIC_JOB_H
#define PERIODIC_JOB_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t period_ms;
    uint32_t next_ms;
} PeriodicJob;

void PeriodicJob_Start(PeriodicJob *job, uint32_t now, uint32_t period_ms);
/* One release at most: missed releases are skipped, never replayed in a burst.
 * All intervals must be < INT32_MAX ms; tick wrap is supported. */
bool PeriodicJob_Take(PeriodicJob *job, uint32_t now,
                      uint32_t *late_ms, uint32_t *skipped);
#endif
