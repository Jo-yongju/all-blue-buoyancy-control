#ifndef TRIAL_RECEIVER_H
#define TRIAL_RECEIVER_H
#include "receiver_command.h"
bool TrialReceiver_Init(void);
void TrialReceiver_Clear(void);
void TrialReceiver_Configure(unsigned switch_channel,bool reverse_switch,bool reverse_ch4);
bool TrialReceiver_ReadCommand(ReceiverCommand *command);
/* ISR only stores bytes / loss markers. ReceiverTask alone calls Step. */
void TrialReceiver_PushFromISR(uint8_t byte,uint32_t stamp);
void TrialReceiver_ErrorFromISR(void);
bool TrialReceiver_Step(void); /* true = budget exhausted with more bytes pending */
typedef struct { uint32_t frames,overflow,uart_errors,peak_bytes; } ReceiverStats;
ReceiverStats TrialReceiver_Stats(void);
#endif
