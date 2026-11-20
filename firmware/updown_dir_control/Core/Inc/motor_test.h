#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H
#include <stdint.h>
#include <stdbool.h>
uint32_t MotorTest_Generation(void);
void MotorTest_Init(void);
void MotorTest_Task(void);
void MotorTest_Tick(uint32_t now);
void MotorTest_Stop(void);
/* ISR entry: stop only; parsing and replies remain in ControlTask. */
void MotorTest_CommunicationStop(bool error);
#endif
