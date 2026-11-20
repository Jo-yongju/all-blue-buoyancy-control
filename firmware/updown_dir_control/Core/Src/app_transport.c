#include "app_transport.h"
#include "FreeRTOS.h"
#include "queue.h"
static StaticQueue_t command_queue_control;
static uint8_t command_queue_storage[APP_COMMAND_DEPTH * sizeof(AppCommandMessage)];
static QueueHandle_t command_queue;
void AppTransport_Init(void)
{
    command_queue=xQueueCreateStatic(APP_COMMAND_DEPTH,sizeof(AppCommandMessage),
                                    command_queue_storage,&command_queue_control);
    configASSERT(command_queue != NULL);
}
bool AppTransport_Send(const AppCommandMessage *message)
{
    return xQueueSend(command_queue,message,0)==pdPASS;
}
bool AppTransport_Receive(AppCommandMessage *message)
{
    return xQueueReceive(command_queue,message,0)==pdPASS;
}
