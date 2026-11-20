/* Compatibility entry for deterministic host regression tests only.
 * Firmware uses AppTasks_Start(), never this polling entry. */
#include "motor_test.h"
#include "control_app.h"
void MotorTest_Task(void)
{
    ControlApp_ConsoleStep();
    ControlApp_Step();
}
