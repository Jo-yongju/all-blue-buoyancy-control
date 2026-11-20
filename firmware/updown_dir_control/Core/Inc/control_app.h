#ifndef CONTROL_APP_H
#define CONTROL_APP_H
/* ControlTask owns commands, ADC, controllers and normal motor writes.
 * ConsoleTask owns line assembly and UART TX; it can only revoke motion. */
void ControlApp_Step(void);
void ControlApp_ConsoleStep(void);
#endif
