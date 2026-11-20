#include "main.h"
#include "trial_receiver.h"
#include "ibus_frame.h"
#include "app_tasks.h"
#include "runtime_policy.h"

/* Every shared multi-field access is a short IRQ-masked copy. Parsing and
 * notification happen outside that copy. No RTOS/driver wait holds a lock. */
#define RX_CAPACITY 256U
typedef struct { uint32_t stamp; uint8_t byte; } RxByte;
static RxByte ring[RX_CAPACITY];
static volatile unsigned head,tail;
static volatile uint32_t generation;
static unsigned switch_channel;
static bool reverse_switch,reverse_ch4,ready;
static ReceiverCommand latest;
static ReceiverStats stats;
/* Parser and parser_generation are owned only by ReceiverTask. */
static IbusFrame frame;
static uint32_t parser_generation;

void TrialReceiver_Configure(unsigned channel,bool reverse_sw,bool reverse_axis)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    switch_channel=channel; reverse_switch=reverse_sw; reverse_ch4=reverse_axis;
    tail=head; ready=false; ++generation;
    if(!mask) __enable_irq();
}
void TrialReceiver_Clear(void) { TrialReceiver_Configure(0,false,false); }
void TrialReceiver_ErrorFromISR(void)
{
    tail=head; ready=false; ++generation; ++stats.uart_errors;
    AppTasks_NotifyReceiverFromISR();
}
void TrialReceiver_PushFromISR(uint8_t byte,uint32_t stamp)
{
    unsigned next=(head+1U)%RX_CAPACITY;
    if(next==tail) {
        /* Drop the whole partial stream, never parse across a missing byte. */
        tail=head; ready=false; ++generation; ++stats.overflow;
        AppTasks_NotifyReceiverFromISR(); return;
    }
    bool wake=head==tail;
    ring[head]=(RxByte){.stamp=stamp,.byte=byte}; head=next;
    unsigned pending=(head+RX_CAPACITY-tail)%RX_CAPACITY;
    if(pending>stats.peak_bytes) stats.peak_bytes=pending;
    if(wake) AppTasks_NotifyReceiverFromISR();
}
bool TrialReceiver_Step(void)
{
    for(unsigned n=0;n<RECEIVER_BYTE_BUDGET;n++) {
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if(head==tail) { if(!mask) __enable_irq(); return false; }
        RxByte b=ring[tail]; tail=(tail+1U)%RX_CAPACITY;
        uint32_t gen=generation;
        unsigned channel=switch_channel;
        bool rs=reverse_switch,rc=reverse_ch4;
        if(!mask) __enable_irq();
        if(gen!=parser_generation) { frame=(IbusFrame){0}; parser_generation=gen; }
        uint16_t channels[14];
        if(!Ibus_Byte(&frame,b.byte,b.stamp,channels)) continue;
        ReceiverCommand command=ReceiverCommand_Decode(channels,b.stamp,channel,rs,rc);
        mask=__get_PRIMASK(); __disable_irq();
        if(gen==generation) { latest=command; ready=true; ++stats.frames; }
        if(!mask) __enable_irq();
    }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool more=head!=tail;
    if(!mask) __enable_irq();
    return more;
}
bool TrialReceiver_ReadCommand(ReceiverCommand *command)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool available=ready;
    if(ready) { *command=latest; ready=false; }
    if(!mask) __enable_irq();
    return available;
}
ReceiverStats TrialReceiver_Stats(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    ReceiverStats copy=stats;
    if(!mask) __enable_irq();
    return copy;
}
