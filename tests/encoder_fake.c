#include "encoder_port.h"
void EncoderPort_Start(void){}
void EncoderPort_Stop(void){}
EncoderRaw EncoderPort_Read(unsigned axis){(void)axis;return (EncoderRaw){0};}
