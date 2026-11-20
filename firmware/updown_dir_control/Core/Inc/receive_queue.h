#ifndef RECEIVE_QUEUE_H
#define RECEIVE_QUEUE_H
#include <stdbool.h>
#include <stdint.h>
#define RECEIVE_QUEUE_BYTES 256U
typedef struct {
    uint8_t data[RECEIVE_QUEUE_BYTES];
    unsigned head, tail, used;
} ReceiveQueue;
/* Caller serializes producer/consumer. On target the consumer masks IRQs for
 * one byte; ISR is the only producer. volatile alone is NOT synchronization. */
bool ReceiveQueue_Push(volatile ReceiveQueue *q, uint8_t byte);
bool ReceiveQueue_Pop(volatile ReceiveQueue *q, uint8_t *byte);
void ReceiveQueue_Clear(volatile ReceiveQueue *q);
#endif
