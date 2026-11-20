#ifndef APP_TASKS_H
#define APP_TASKS_H
#include <stddef.h>
#include <stdint.h>
void AppTasks_Start(void);
void AppTasks_NotifyReceiverFromISR(void);
void AppTasks_NotifyImuFromISR(void);
void AppTasks_NotifyImu(void);
void AppTasks_DelayMs(uint32_t milliseconds);
void AppTasks_FormatStats(char *text, size_t size);
void AppTasks_Fatal(const char *file, unsigned line);
#endif
