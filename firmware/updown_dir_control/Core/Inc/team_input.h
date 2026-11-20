#ifndef TEAM_INPUT_H
#define TEAM_INPUT_H
#include "receiver_command.h"
/* Fixed agreement: CH6/SwC=1000 UP, 1500 HOVER, 2000 DOWN;
 * CH4/LH=1000 nose down, 1500 level, 2000 nose up. */
ReceiverCommand TeamInput_Radio(uint16_t lh, uint16_t swc, bool failsafe,
                                uint32_t received_ms);
/* Team Quaternion.c already returns degrees. Normalize installed mounting
 * once, before publication. sign must be +1 or -1; level is measured. */
bool TeamInput_Pitch(float raw_deg, float level_deg, int sign, float *pitch_deg);
#endif
