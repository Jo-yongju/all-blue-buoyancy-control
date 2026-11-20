#include "periodic_job.h"
#include "console_queue.h"
#include "receive_queue.h"
#include "motor_command.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    PeriodicJob job;
    uint32_t late, skipped;
    PeriodicJob_Start(&job, 100, 10);
    assert(!PeriodicJob_Take(&job, 109, &late, &skipped));
    assert(PeriodicJob_Take(&job, 112, &late, &skipped) && late == 2 && skipped == 0);
    assert(job.next_ms == 120); /* phase stays fixed despite late work */
    assert(PeriodicJob_Take(&job, 143, &late, &skipped) && skipped == 2 && late == 23);
    assert(job.next_ms == 150);
    assert(!PeriodicJob_Take(&job, 143, &late, &skipped)); /* no catch-up burst */
    PeriodicJob_Start(&job, UINT32_MAX - 5, 10);
    assert(!PeriodicJob_Take(&job, 3, &late, &skipped));
    assert(PeriodicJob_Take(&job, 4, &late, &skipped) && late == 0);

    ConsoleQueue q = {0};
    uint8_t byte;
    char large[700]; memset(large, 'x', sizeof(large)-1); large[699] = 0;
    assert(!ConsoleQueue_Push(&q, large, true)); /* preserves reply reserve */
    assert(q.used == 0 && q.dropped == 1);
    assert(ConsoleQueue_Push(&q, large, false));
    unsigned used = q.used;
    assert(!ConsoleQueue_Push(&q, large, false));
    assert(q.used == used); /* failed push cannot enqueue partial message */
    for (unsigned i=0; i<used; ++i) {
        assert(ConsoleQueue_Peek(&q, &byte) && byte == 'x'); ConsoleQueue_Pop(&q);
    }
    for (unsigned repeat=0; repeat<2000; ++repeat) {
        assert(ConsoleQueue_Push(&q, "abc", false));
        for (unsigned i=0;i<3;++i) {
            assert(ConsoleQueue_Peek(&q,&byte) && byte == (uint8_t)"abc"[i]); ConsoleQueue_Pop(&q);
        }
    }
    assert(q.used == 0 && !ConsoleQueue_Peek(&q,&byte));
    volatile ReceiveQueue rx = {0};
    for (unsigned i=0; i<RECEIVE_QUEUE_BYTES; ++i) assert(ReceiveQueue_Push(&rx,(uint8_t)i));
    assert(!ReceiveQueue_Push(&rx, 99));
    for (unsigned i=0; i<RECEIVE_QUEUE_BYTES; ++i) {
        assert(ReceiveQueue_Pop(&rx,&byte) && byte == (uint8_t)i);
    }
    assert(!ReceiveQueue_Pop(&rx,&byte));
    for (unsigned i=0;i<2000;++i) {
        assert(ReceiveQueue_Push(&rx,(uint8_t)i));
        assert(ReceiveQueue_Pop(&rx,&byte) && byte == (uint8_t)i);
    }
    assert(ReceiveQueue_Push(&rx,1)); ReceiveQueue_Clear(&rx);
    assert(!ReceiveQueue_Pop(&rx,&byte));
    assert(MotorCommand_Parse("STATS").type == CMD_STATS);
    assert(MotorCommand_Parse("TRACE ON").type == CMD_TRACE_ON);
    assert(MotorCommand_Parse("TRACE OFF").type == CMD_TRACE_OFF);
    assert(MotorCommand_Parse("TRACE ON garbage").type == CMD_INVALID);
    puts("PASS fixed-phase scheduler, skipped releases, wrap, bounded queue, whole-line drops, telemetry reserve and diagnostics parser");
}
