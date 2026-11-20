#pragma once
#include <stdbool.h>
#include <stdint.h>
void LiveIO_InitOutputs(void);
void LiveIO_Off(void);
void LiveIO_Jog(unsigned axis, int percent);
/* Caller owns stop permission and enforces reversal dead time. No waits. */
void LiveIO_SingleWrite(unsigned axis, int percent);
void LiveIO_Write(float front,float rear,uint32_t now);
bool LiveIMU_Init(void);
bool LiveIMU_Pending(void);
bool LiveIMU_Sample(float *pitch);
