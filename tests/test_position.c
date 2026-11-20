#include "position_control.h"
#include "motor_command.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static PositionConfig config(void) {
    PositionConfig c={
        .calibration={{100,1000,3900},{0,30,60},true}, .motor_sign=1,
        .safe_min_mm=1,.safe_max_mm=59,.kp=8,.ki=2,.max_pwm_percent=30,
        .tolerance_mm=.4f,.target_rate_mm_s=2,.max_sensor_speed_mm_s=15,
        .stall_ms=1500,.max_move_ms=60000,.settle_ms=200};
    return c;
}
static uint16_t raw_at(float mm) {
    return (uint16_t)lroundf(mm<=30?100+mm*900/30:1000+(mm-30)*2900/30);
}
int main(void) {
    PositionConfig c=config();float mm;
    assert(PositionConfig_Valid(&c));
    assert(SliderCalibration_Position(&c.calibration,550,&mm)&&fabsf(mm-15)<.001f);
    assert(SliderCalibration_Position(&c.calibration,2450,&mm)&&fabsf(mm-45)<.001f);
    assert(SliderCalibration_Position(&c.calibration,1000,&mm)&&mm==30);
    assert(!SliderCalibration_Position(&c.calibration,99,&mm));
    assert(!SliderCalibration_Position(&c.calibration,3901,&mm));
    c.calibration.adc[0]=3900;c.calibration.adc[2]=100;
    assert(SliderCalibration_Position(&c.calibration,2450,&mm)&&fabsf(mm-15)<.001f);
    assert(SliderCalibration_Position(&c.calibration,550,&mm)&&fabsf(mm-45)<.001f);
    c.calibration.adc[1]=3950;assert(!SliderCalibration_Valid(&c.calibration));
    c=config();c.calibration.confirmed=false;assert(!PositionConfig_Valid(&c));
    c=config();c.motor_sign=0;assert(!PositionConfig_Valid(&c));
    c=config();c.calibration.mm[1]=NAN;assert(!PositionConfig_Valid(&c));
    c=config();c.safe_max_mm=60;assert(!PositionConfig_Valid(&c));
    c=config();c.ki=NAN;assert(!PositionConfig_Valid(&c));

    PositionControl s;c=config();
    assert(Position_Start(&s,&c,0,1000,1000)==POSITION_TARGET_ERROR);
    assert(Position_Start(&s,&c,NAN,1000,1000)==POSITION_TARGET_ERROR);
    assert(Position_Start(&s,&c,40,0,1000)==POSITION_SENSOR_ERROR);
    assert(Position_Start(&s,&c,40,100,1000)==POSITION_LIMIT_ERROR);
    Position_Start(&s,&c,40,1000,1000);
    assert(Position_Update(&s,&c,false,1000,1010)==POSITION_SENSOR_ERROR&&s.pwm_percent==0);
    Position_Start(&s,&c,40,1000,1000);
    assert(Position_Update(&s,&c,true,1000,1051)==POSITION_TIMING_ERROR&&s.pwm_percent==0);
    Position_Start(&s,&c,40,1000,1000);
    assert(Position_Update(&s,&c,true,3900,1010)==POSITION_LIMIT_ERROR);
    Position_Start(&s,&c,40,1000,1000);
    assert(Position_Update(&s,&c,true,2000,1010)==POSITION_JUMP_ERROR);
    Position_Start(&s,&c,40,1000,1000);
    for(uint32_t t=1010;t<3000&&s.status==POSITION_RUNNING;t+=10) Position_Update(&s,&c,true,1000,t);
    assert(s.status==POSITION_STALL_ERROR&&s.pwm_percent==0);
    assert(Position_Update(&s,&c,true,1000,3010)==POSITION_STALL_ERROR); /* latched */
    c=config();c.max_move_ms=1000;
    Position_Start(&s,&c,40,1000,1000);
    for(uint32_t t=1010;t<=2000;t+=10) Position_Update(&s,&c,true,1000,t);
    assert(s.status==POSITION_TIMEOUT_ERROR&&s.pwm_percent==0);
    c=config();Position_Start(&s,&c,30,1000,UINT32_MAX-100);
    for(uint32_t i=1;i<=22;i++) Position_Update(&s,&c,true,1000,(UINT32_MAX-100)+i*10);
    assert(s.status==POSITION_DONE&&s.pwm_percent==0);

    /* Simple actuator model: prove both commands converge and PWM stays bounded.
     * This is a software test, not measured piston/pressure performance. */
    for(int dir=-1;dir<=1;dir+=2) {
        c=config();c.motor_sign=dir;float actual=30;
        Position_Start(&s,&c,35,raw_at(actual),0);
        for(uint32_t t=10;t<=20000&&s.status==POSITION_RUNNING;t+=10) {
            Position_Update(&s,&c,true,raw_at(actual),t);
            assert(abs(s.pwm_percent)<=30);
            actual+=s.pwm_percent*dir*.15f*.01f;
        }
        assert(s.status==POSITION_DONE&&fabsf(actual-35)<.5f);
        Position_Start(&s,&c,25,raw_at(actual),30000);
        for(uint32_t t=30010;t<=50000&&s.status==POSITION_RUNNING;t+=10) {
            Position_Update(&s,&c,true,raw_at(actual),t);
            actual+=s.pwm_percent*dir*.15f*.01f;
        }
        assert(s.status==POSITION_DONE&&fabsf(actual-25)<.5f);
    }
    assert(MotorCommand_Parse("CAL").type==CMD_CAL);
    MotorCommand cmd=MotorCommand_Parse("MOVE 12.5");
    assert(cmd.type==CMD_MOVE&&cmd.target_mm==12.5f);
    const char *bad[]={"MOVE nan","MOVE inf","MOVE -1","MOVE","MOVE 1 extra","MOVE 1e999","CAL x"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(MotorCommand_Parse(bad[i]).type==CMD_INVALID);
    puts("PASS: 3-point increasing/decreasing calibration, invalid configs, target/sensor/limit/jump/timing/stall/timeout, tick wrap, bidirectional position convergence, parser");
}
