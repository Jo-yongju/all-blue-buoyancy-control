#ifndef APP_TRANSPORT_H
#define APP_TRANSPORT_H
#include <stdbool.h>
#include <stdint.h>
#define APP_COMMAND_LENGTH 64U
#define APP_COMMAND_DEPTH 4U
typedef struct { uint32_t epoch; char line[APP_COMMAND_LENGTH]; } AppCommandMessage;
void AppTransport_Init(void);
bool AppTransport_Send(const AppCommandMessage *message);
bool AppTransport_Receive(AppCommandMessage *message);
#endif
