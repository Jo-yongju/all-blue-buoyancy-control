#ifndef INPUT_TRIAL_H
#define INPUT_TRIAL_H
#include "syringe_control.h"
#include "receiver_command.h"
#include <stddef.h>

typedef enum { TRIAL_OFF, TRIAL_SIM, TRIAL_RC } TrialSource;
typedef struct {
    TrialSource source;
    SyControl control;
    SyInput input;
    unsigned switch_channel;
    bool fresh_sim, fixed_adc;
    float model_mm[2];
    uint16_t channels[14];
    bool have_frame;
    uint32_t frame_ms, tick_ms;
} InputTrial;

void Trial_Start(InputTrial *t, TrialSource source, unsigned channel, uint32_t now);
void Trial_ReceiverCommand(InputTrial *t, const ReceiverCommand *command);
void Trial_Frame(InputTrial *t, const uint16_t channels[14], uint32_t now);
void Trial_Tick(InputTrial *t, uint32_t now);
bool Trial_Command(InputTrial *t, const char *line, uint32_t now, char *reply, size_t capacity);
#endif
