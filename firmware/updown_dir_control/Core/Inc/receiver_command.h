#ifndef RECEIVER_COMMAND_H
#define RECEIVER_COMMAND_H
#include "syringe_control.h"
typedef struct {
    uint16_t channels[14];
    uint32_t received_ms;
    SyMode requested;
    float stick;
    bool command_valid;
} ReceiverCommand;
/* Generic bench mapping. Confirmed team mapping is in team_input.h. */
ReceiverCommand ReceiverCommand_Decode(const uint16_t channels[14], uint32_t stamp,
                                      unsigned switch_channel, bool reverse_switch, bool reverse_ch4);
#endif
