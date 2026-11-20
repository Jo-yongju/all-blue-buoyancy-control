#ifndef RUNTIME_PORT_H
#define RUNTIME_PORT_H
#include <stdbool.h>
#include <stdint.h>

/* LPUART1/COM1 has ONE owner after BSP_COM_Init: this console.
 * Do not mix HAL blocking/IT/DMA UART transmission with this port. */
bool RuntimePort_WriteByte(uint8_t byte);
bool RuntimePort_ReadByte(uint8_t *byte);
void RuntimePort_StartReceiver(void);
void RuntimePort_UartIrq(void);
void RuntimePort_InitClock(void);
uint32_t RuntimePort_Cycles(void);
uint32_t RuntimePort_ElapsedUs(uint32_t since);
bool RuntimePort_ClockReady(void);
#endif
