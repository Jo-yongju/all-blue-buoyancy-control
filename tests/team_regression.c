#include "team_input.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static SyConfig config(void) {
    SyConfig c={.enable=true,.angle_enable=true,.radio_auto_resume=true,
        .position_kp=.03f,.position_ki=.005f,.pwm_limit=.3f,.position_rate_mm_s=5,
        .pitch_kp=.06f,.pitch_ki=.005f,.pitch_kd=.02f,.pitch_sign=-1,
        .max_angle_deg=5,.angle_rate_deg_s=10,.pitch_stop_deg=.5f,
        .pitch_resume_deg=.8f,.pitch_stop_rate=1,.pitch_resume_rate=2};
    for(unsigned i=0;i<2;i++) {
        c.axis[i]=(SyAxis){.cal={{0,529,4095},{0,29.24f,58.48f},true},
            .motor_sign=1,.stroke_mm=58.48f,.volume_span_cc=120};
        c.base_cc[0][i]=10;c.base_cc[1][i]=60;c.base_cc[2][i]=110;
    }
    return c;
}
static void tick(SyControl *s,SyConfig *c,SyInput *in,unsigned t) {
    in->radio_ms=t;in->imu_ms=t;Sy_Tick(s,c,in,t,.01f);
}
int main(void) {
    ReceiverCommand r=TeamInput_Radio(2000,1500,false,123);
    assert(r.command_valid && r.requested==SY_HOVER && r.stick==1 && r.received_ms==123);
    assert(TeamInput_Radio(1000,1000,false,1).requested==SY_ASCEND);
    assert(TeamInput_Radio(1000,2000,false,1).requested==SY_DESCEND);
    assert(TeamInput_Radio(1000,1500,false,1).stick==-1);
    assert(!TeamInput_Radio(1500,1500,true,1).command_valid);
    assert(!TeamInput_Radio(65535,1500,false,1).command_valid);
    assert(!TeamInput_Radio(1500,0,false,1).command_valid);
    float p;
    assert(TeamInput_Pitch(-179,179,1,&p) && p==2);
    assert(TeamInput_Pitch(-179,179,-1,&p) && p==-2);
    assert(!TeamInput_Pitch(NAN,0,1,&p));assert(!TeamInput_Pitch(0,0,0,&p));
    SyConfig c=config();assert(Sy_Validate(&c));
    SyInput in={.radio_ok=true,.adc_ok=true,.imu_ok=true,.requested=SY_HOVER,.adc={529,529}};
    SyControl s;Sy_Init(&s);
    for(unsigned t=0;t<=600;t+=10)tick(&s,&c,&in,t);
    assert(s.armed && s.mode==SY_HOVER && s.pwm[0]==0 && s.pwm[1]==0);
    in.stick=1;
    for(unsigned t=610;t<=900;t+=10)tick(&s,&c,&in,t);
    assert(s.pwm[0]<0 && s.pwm[1]>0); /* front expels / rear fills: nose up */
    in.pitch_deg=5;
    for(unsigned t=910;t<=1900;t+=10)tick(&s,&c,&in,t);
    assert(s.pitch_holding && s.pwm[0]==0 && s.pwm[1]==0);
    in.requested=SY_DESCEND;
    for(unsigned t=1910;t<=2200;t+=10)tick(&s,&c,&in,t);
    assert(s.pwm[0]>0 && s.pwm[1]>0); /* CH4 ignored outside HOVER */
    in.radio_ok=false;tick(&s,&c,&in,2210);
    assert(s.fault==SY_RADIO && s.pwm[0]==0 && s.pwm[1]==0);
    in.radio_ok=true;
    for(unsigned t=2220;t<2420;t+=10){tick(&s,&c,&in,t);assert(s.mode==SY_FAULT);}
    tick(&s,&c,&in,2420);assert(s.armed && s.mode==SY_DESCEND);
    Sy_Tick(&s,&c,&in,2800,.01f);assert(s.fault==SY_RADIO); /* stale frame */
    in.imu_ok=false;in.requested=SY_HOVER;
    for(unsigned t=2810;t<=3200;t+=10){tick(&s,&c,&in,t);assert(s.mode==SY_FAULT);}
    in.imu_ok=true;
    for(unsigned t=3210;t<=3430;t+=10)tick(&s,&c,&in,t);
    assert(s.mode==SY_HOVER);
    Sy_Fail(&s,SY_STALL);
    for(unsigned t=3440;t<=3800;t+=10){tick(&s,&c,&in,t);assert(s.mode==SY_FAULT);}
    puts("PASS team CH4/CH6 mapping, invalid/failsafe, degree wrapping, HOVER angle PWM/stop, non-HOVER ignores CH4, radio timeout + held-switch recovery, missing IMU blocks recovery, stall not auto-retried");
    return 0;
}
