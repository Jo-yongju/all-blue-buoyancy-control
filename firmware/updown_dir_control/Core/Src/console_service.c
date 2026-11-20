#include "main.h"
#include "console_service.h"
#include "console_queue.h"
#include "control_app.h"
#include "app_transport.h"
#include "motor_test.h"
#include "runtime_port.h"
#include "runtime_policy.h"
#include <string.h>
static ConsoleQueue tx_queue;
void ConsoleService_Reply(const char *text,bool telemetry)
{
    /* Bounded memory copy only; never hold this guard around I/O waits. */
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    (void)ConsoleQueue_Push(&tx_queue,text,telemetry);
    if(!mask) __enable_irq();
}
ConsoleServiceStats ConsoleService_ReadStats(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    ConsoleServiceStats copy={.peak=tx_queue.peak,.dropped=tx_queue.dropped};
    if(!mask) __enable_irq();
    return copy;
}
static void ServiceTx(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    for (unsigned i = 0; i < CONSOLE_TX_BUDGET; ++i) {
        uint8_t byte;
        if (!ConsoleQueue_Peek(&tx_queue, &byte) || !RuntimePort_WriteByte(byte)) break;
        ConsoleQueue_Pop(&tx_queue);
    }
    if (!mask) __enable_irq();
}

/* ConsoleTask exclusively owns line assembly. */
static unsigned console_length;
static bool console_discard;
static char console_line[APP_COMMAND_LENGTH];
static uint32_t console_epoch;

void ControlApp_ConsoleStep(void)
{
    for(unsigned i=0;i<CONSOLE_RX_BUDGET;i++) {
        if(console_epoch!=MotorTest_Generation()) {
            console_discard=console_length!=0;
            console_length=0; console_epoch=MotorTest_Generation();
        }
        uint8_t ch;
        if(!RuntimePort_ReadByte(&ch)) break;
        if(ch=='!') {
            MotorTest_CommunicationStop(false);
            console_length=0; console_discard=true; console_epoch=MotorTest_Generation();
            break;
        }
        if(ch=='\r'||ch=='\n') {
            if(!console_discard && console_length) {
                console_line[console_length]=0;
                if(!strcmp(console_line,"STOP")) {
                    MotorTest_CommunicationStop(false);
                } else {
                    AppCommandMessage message={.epoch=console_epoch};
                    memcpy(message.line,console_line,console_length+1);
                    if(!AppTransport_Send(&message)) MotorTest_CommunicationStop(true);
                }
            }
            console_length=0; console_discard=false; console_epoch=MotorTest_Generation();
            break;
        }
        if(!console_discard) {
            if(ch<32||ch>126||console_length>=sizeof console_line-1) {
                MotorTest_CommunicationStop(true);
                console_length=0; console_discard=true; console_epoch=MotorTest_Generation();
            } else console_line[console_length++]=(char)ch;
        }
    }
    ServiceTx();
}

