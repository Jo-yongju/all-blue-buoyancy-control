#ifndef CONSOLE_QUEUE_H
#define CONSOLE_QUEUE_H
#include <stdbool.h>
#include <stdint.h>
#define CONSOLE_QUEUE_BYTES 1024U
#define CONSOLE_REPLY_RESERVE 384U
typedef struct {
    char data[CONSOLE_QUEUE_BYTES];
    unsigned head, tail, used, peak;
    uint32_t dropped;
} ConsoleQueue;
/* Not intrinsically thread/ISR-safe. Caller must serialize access. Whole messages are accepted
 * or dropped, so overload cannot splice two partial response lines together. */
bool ConsoleQueue_Push(ConsoleQueue *q, const char *text, bool telemetry);
bool ConsoleQueue_Peek(const ConsoleQueue *q, uint8_t *byte);
void ConsoleQueue_Pop(ConsoleQueue *q);
#endif
