#include "team_input.h"
#include <math.h>

ReceiverCommand TeamInput_Radio(uint16_t lh,uint16_t swc,bool failsafe,uint32_t stamp)
{
    uint16_t ch[14];
    for (unsigned i=0;i<14;i++) ch[i]=1500;
    ch[3]=lh; ch[5]=swc;
    ReceiverCommand c=ReceiverCommand_Decode(ch,stamp,6,false,false);
    if (failsafe) c.command_valid=false;
    return c;
}

bool TeamInput_Pitch(float raw,float level,int sign,float *pitch)
{
    if (!pitch || !isfinite(raw) || !isfinite(level) ||
        fabsf(raw)>360 || fabsf(level)>360 || (sign!=1 && sign!=-1)) return false;
    float delta=fmodf(raw-level,360.0f);
    if (delta>180) delta-=360;
    if (delta< -180) delta+=360;
    *pitch=delta*sign;
    return true;
}
