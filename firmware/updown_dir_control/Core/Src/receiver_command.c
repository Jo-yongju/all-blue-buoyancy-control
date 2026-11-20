#include "receiver_command.h"
#include <string.h>
ReceiverCommand ReceiverCommand_Decode(const uint16_t ch[14],uint32_t stamp,
                                      unsigned channel,bool reverse_switch,bool reverse_ch4)
{
    ReceiverCommand c={.received_ms=stamp,.requested=SY_HOVER};
    memcpy(c.channels,ch,sizeof c.channels);
    if (!channel || channel>14 || channel==4) return c;
    for (unsigned i=0;i<14;i++) if(ch[i]<900 || ch[i]>2100) return c;
    unsigned sw=ch[channel-1];
    c.requested=sw<1300 ? (reverse_switch?SY_DESCEND:SY_ASCEND) :
                sw>1700 ? (reverse_switch?SY_ASCEND:SY_DESCEND) : SY_HOVER;
    int v=(int)ch[3]-1500;
    if(v>500) v=500;
    if(v< -500) v=-500;
    c.stick=(float)v/500.0f*(reverse_ch4?-1.0f:1.0f);
    c.command_valid=true;
    return c;
}
