#include "receive_queue.h"

bool ReceiveQueue_Push(volatile ReceiveQueue *q, uint8_t byte)
{
    if (q->used == RECEIVE_QUEUE_BYTES) return false;
    q->data[q->head] = byte;
    q->head = (q->head + 1U) % RECEIVE_QUEUE_BYTES;
    ++q->used;
    return true;
}

bool ReceiveQueue_Pop(volatile ReceiveQueue *q, uint8_t *byte)
{
    if (q->used == 0) return false;
    *byte = q->data[q->tail];
    q->tail = (q->tail + 1U) % RECEIVE_QUEUE_BYTES;
    --q->used;
    return true;
}

void ReceiveQueue_Clear(volatile ReceiveQueue *q)
{
    q->head = q->tail = q->used = 0;
}
