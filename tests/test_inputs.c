#include "input_trial.h"
#include "ibus_frame.h"
#include "live_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static void cmd(InputTrial *t,const char *line,uint32_t now)
{
    char reply[200];
    assert(Trial_Command(t,line,now,reply,sizeof reply));
    assert(!strncmp(reply,"OK",2));
}

static void ibus_test(void)
{
    IbusFrame parser={0};
    uint8_t frame[32]={0x20,0x40};
    for (int i=0;i<14;i++) {frame[2+2*i]=0xdc;frame[3+2*i]=5;}
    uint16_t sum=0xffff,out[14]={0};
    for (int i=0;i<30;i++) sum-=frame[i];
    frame[30]=(uint8_t)sum;frame[31]=(uint8_t)(sum>>8);
    for (int i=0;i<32;i++) assert(Ibus_Byte(&parser,frame[i],8,out)==(i==31));
    assert(out[3]==1500);
    frame[2]^=1;
    for (int i=0;i<32;i++) assert(!Ibus_Byte(&parser,frame[i],16,out));
}
int main(void)
{
    ibus_test();
    SyConfig c=live_cfg;
    /* Bench test values only. The shipping image must refuse physical ARM. */
    assert(!c.enable && !Sy_Validate(&c));
    for (int i=0;i<2;i++) {
        c.axis[i].cal=(SliderCalibration){{100,620,3800},{0,29.24f,58.48f},true};
        c.axis[i].motor_sign=1;
        c.axis[i].guard_mm=60;c.axis[i].guard_adc=3970;
    }
    c.enable=true;
    assert(Sy_Validate(&c));
    SyControl live={0};
    SyInput in={.radio_ok=true,.adc_ok=true,.imu_ok=true,.requested=SY_HOVER,.adc={620,620}};
    for(unsigned t=10;t<=1500;t+=10){
        in.radio_ms=in.imu_ms=t;
        if(t==600) in.pitch_deg=3; /* nose already high -> more water in front */
        Sy_Tick(&live,&c,&in,t,.01f);
    }
    assert(live.armed && live.pwm[0]>0 && live.pwm[1]<0);
    in.radio_ms=1610;
    Sy_Tick(&live,&c,&in,1610,.01f);
    assert(live.fault==SY_IMU && live.pwm[0]==0 && live.pwm[1]==0);
    Sy_Init(&live);in.adc[0]=3970;in.radio_ms=in.imu_ms=1620;
    Sy_Tick(&live,&c,&in,1620,.01f);
    assert(live.fault==SY_LIMIT);
    c.axis[1].guard_mm=0; assert(!Sy_Validate(&c));
    puts("PASS: live interlock, 60cc guard, nose-up correction, stale pitch stop, travel stop");
    InputTrial sim={0},rc={0};
    cmd(&sim,"MODE SIM",0);cmd(&rc,"MODE RC 7",0);
    uint16_t channels[14];
    for (unsigned i=0;i<14;i++) channels[i]=1500;
    for (unsigned t=10;t<=1800;t+=10) {
        if (t==620) {cmd(&sim,"SW DOWN",t);channels[6]=2000;}
        if (t==1000) {cmd(&sim,"SW HOVER",t);cmd(&sim,"STICK 100",t);channels[6]=1500;channels[3]=2000;}
        if (t==1500) {cmd(&sim,"SW UP",t);channels[6]=1000;}
        Trial_Frame(&rc,channels,t);
        Trial_Tick(&rc,t);Trial_Tick(&sim,t);
        assert(rc.control.mode==sim.control.mode && rc.control.fault==SY_OK);
        for (int i=0;i<2;i++) assert(rc.control.target[i]==sim.control.target[i]);
        if(t==610) assert(rc.control.armed && fabsf(rc.control.position[0]-29.24f)<.01f &&
                           fabsf(rc.control.position[1]-29.24f)<.01f);
        if(t==990) assert(rc.control.target[0]>29.24f && rc.control.target[0]==rc.control.target[1]);
        if(t==1490) assert(rc.control.pwm[0]>0 && rc.control.pwm[1]<0);
    }
    for (unsigned t=1810;t<=2110;t+=10) Trial_Tick(&rc,t);
    assert(rc.control.fault==SY_RADIO && rc.control.pwm[0]==0 && rc.control.pwm[1]==0);
    cmd(&sim,"MODE RC",2200);Trial_Frame(&sim,channels,2210);Trial_Tick(&sim,2210);
    assert(!sim.control.armed && sim.have_frame);
    cmd(&sim,"MODE SIM",2300);
    for (unsigned t=2310;t<=2900;t+=10) Trial_Tick(&sim,t);
    cmd(&sim,"ADC 630 3350",2900);cmd(&sim,"SW DOWN",2900);
    for (unsigned t=2910;t<6000;t+=10) Trial_Tick(&sim,t);
    assert(sim.control.fault==SY_STALL && sim.control.pwm[0]==0 && sim.control.pwm[1]==0);
    cmd(&sim,"STOP",6000);assert(sim.source==TRIAL_OFF && !sim.control.armed);
    cmd(&sim,"MODE SIM",6000);
    for (unsigned t=6010;t<=6600;t+=10) Trial_Tick(&sim,t);
    cmd(&sim,"SW DOWN",6600);
    for (unsigned t=6610;t<=21600;t+=10) Trial_Tick(&sim,t);
    assert(sim.control.mode==SY_DESCEND && sim.control.fault==SY_OK);
    for (unsigned i=0;i<2;i++) {
        assert(fabsf(sim.control.target[i]-58.48f)<0.001f);
        assert(fabsf(sim.control.position[i]-58.48f)<0.25f);
        assert(sim.control.pwm[i]==0);
    }
    puts("PASS: 60 cc per syringe, both axes settled");
    puts("PASS: iBUS checksum, SIM/RC equivalence, nonlinear midpoint, RC discovery, neutral arming, stale radio, stall, stop");
}
