#ifndef APP_INPUTS_H
#define APP_INPUTS_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    float pitch_deg;
    uint32_t sampled_ms;
    bool valid, initialized, initializing;
} AppImuSnapshot;
void AppInputs_RequestImu(bool enabled);
void AppInputs_Step(void);
bool AppInputs_ImuPending(void);
AppImuSnapshot AppInputs_ReadImu(void);
#endif
