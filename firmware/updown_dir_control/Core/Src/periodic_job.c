#include "periodic_job.h"

void PeriodicJob_Start(PeriodicJob *job, uint32_t now, uint32_t period_ms)
{
    job->period_ms = period_ms;
    job->next_ms = now + period_ms;
}

bool PeriodicJob_Take(PeriodicJob *job, uint32_t now,
                      uint32_t *late_ms, uint32_t *skipped)
{
    if (job->period_ms == 0 || (int32_t)(now - job->next_ms) < 0) return false;
    *late_ms = now - job->next_ms;
    *skipped = *late_ms / job->period_ms;
    job->next_ms += (*skipped + 1U) * job->period_ms;
    return true;
}
