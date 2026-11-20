#pragma once
#include "encoder_observer.h"
/* Reserved observation pins: front PC0/PC1, rear PC2/PC3 (A/B).
 * No pin is claimed at boot. Verify encoder signal voltage before ENC ON. */
void EncoderPort_Start(void);
void EncoderPort_Stop(void);
EncoderRaw EncoderPort_Read(unsigned axis);
