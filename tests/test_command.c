#include "motor_command.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    assert(MotorCommand_Parse("STOP").type==CMD_STOP);
    assert(MotorCommand_Parse("ARM").type==CMD_ARM);
    assert(MotorCommand_Parse("STATUS").type==CMD_STATUS);
    assert(MotorCommand_Parse("HELP").type==CMD_HELP);
    assert(MotorCommand_Parse("SENSOR").type==CMD_SENSOR);
    assert(MotorCommand_Parse("SENSOR extra").type==CMD_INVALID);
    MotorCommand c=MotorCommand_Parse("JOG 30 300");
    assert(c.type==CMD_JOG&&c.percent==30&&c.duration_ms==300);
    c=MotorCommand_Parse("JOG -100 1000");
    assert(c.type==CMD_JOG&&c.percent==-100&&c.duration_ms==1000);
    c=MotorCommand_Parse("JOG +1 20");
    assert(c.type==CMD_JOG&&c.percent==1&&c.duration_ms==20);
    c=MotorCommand_Parse("JOG 100 15000");
    assert(c.type==CMD_JOG&&c.duration_ms==15000);
    c=MotorCommand_Parse("JOG -100 20000");
    assert(c.type==CMD_JOG&&c.duration_ms==20000);
    const char *bad[]={"", "JOG", "JOG 30", "JOG 0 300", "JOG 101 300", "JOG -101 300",
      "JOG 30 0","JOG 30 -1","JOG 30 20001","JOG 30 300 extra","JOG 30.5 300",
      "JOG 30x 300","JOG abc 300","JOG 99999999999999999999999999 300",
      "JOG 30 999999999999999999999999","ARM extra","STOP now",NULL};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(MotorCommand_Parse(bad[i]).type==CMD_INVALID);
    puts("PASS motor command parser: signed duty, duration bounds, malformed input, overflow");
}
