#include "console_queue.h"
#include <string.h>

bool ConsoleQueue_Push(ConsoleQueue *q, const char *text, bool telemetry)
{
    size_t length = strlen(text);
    unsigned reserve = telemetry ? CONSOLE_REPLY_RESERVE : 0;
    if (length > CONSOLE_QUEUE_BYTES - q->used ||
        reserve > CONSOLE_QUEUE_BYTES - q->used - length) {
        ++q->dropped;
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        q->data[q->head] = text[i];
        q->head = (q->head + 1U) % CONSOLE_QUEUE_BYTES;
    }
    q->used += (unsigned)length;
    if (q->used > q->peak) q->peak = q->used;
    return true;
}

bool ConsoleQueue_Peek(const ConsoleQueue *q, uint8_t *byte)
{
    if (q->used == 0) return false;
    *byte = (uint8_t)q->data[q->tail];
    return true;
}

void ConsoleQueue_Pop(ConsoleQueue *q)
{
    if (q->used == 0) return;
    q->tail = (q->tail + 1U) % CONSOLE_QUEUE_BYTES;
    --q->used;
}
