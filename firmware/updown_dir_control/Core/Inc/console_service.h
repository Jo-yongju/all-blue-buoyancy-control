#ifndef CONSOLE_SERVICE_H
#define CONSOLE_SERVICE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { unsigned peak; uint32_t dropped; } ConsoleServiceStats;
void ConsoleService_Reply(const char *text,bool telemetry);
ConsoleServiceStats ConsoleService_ReadStats(void);
#endif
