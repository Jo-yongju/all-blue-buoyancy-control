#define main earlier_team_regression
#include "team_regression.c"
#undef main
#include "live_config.h"
static void seed(SyControl *s,SyConfig *c,SyInput *in,unsigned i,unsigned raw,unsigned t) {
    float mm=0;assert(SliderCalibration_Position(&c->axis[i].cal,raw,&mm));
    in->adc[i]=raw;SignalGuard_Seed(&s->signal[i],raw,mm,t);s->position[i]=mm;
}
static unsigned raw_for(const SyAxis *a,float mm) {
    unsigned k=mm<=a->cal.mm[1]?0:1;
    float f=(mm-a->cal.mm[k])/(a->cal.mm[k+1]-a->cal.mm[k]);
    return (unsigned)lroundf(a->cal.adc[k]+f*((int)a->cal.adc[k+1]-(int)a->cal.adc[k]));
}
int main(void) {
    SyConfig c=live_cfg;assert(c.fixed_position && c.axis[1].motor_sign==-1);
    assert(Sy_Validate(&c));assert(c.pwm_limit==.5f);c.enable=true;
    SyInput in={.radio_ok=true,.adc_ok=true,.imu_ok=true,.requested=SY_HOVER,.adc={500,560}};
    SyControl s;Sy_Init(&s);
    for(unsigned t=0;t<=600;t+=10)tick(&s,&c,&in,t);
    assert(s.armed && !s.hover_position_ready && s.pwm[0]==-.5f && s.pwm[1]==.5f);
    in.stick=1;in.adc[0]=522;tick(&s,&c,&in,610);
    assert(s.position_complete[0] && s.pwm[0]==0 && s.pwm[1]==.5f && !s.hover_position_ready);
    in.adc[0]=515;in.adc[1]=533;tick(&s,&c,&in,620);
    assert(s.pwm[0]==0 && s.pwm[1]==0 && s.hover_position_ready); /* within middle tolerance */
    for(unsigned t=630;t<=1000;t+=10)tick(&s,&c,&in,t);
    assert(s.fault==SY_OK && s.pwm[0]>0 && s.pwm[1]<0 && fabsf(s.pwm[0])<1);
    /* HOVER stays in pitch stage even after feedback moves away from center. */
    seed(&s,&c,&in,0,0,1010);
    for(unsigned t=1010;t<=11000;t+=10) {
        tick(&s,&c,&in,t);
        assert(s.fault==SY_OK && s.pitch_limited && s.pwm[0]==0 && s.pwm[1]==0);
        assert(s.pitch_integral==0 && s.hover_position_ready);
    }
    in.stick=-1;
    for(unsigned t=11010;t<=12200;t+=10)tick(&s,&c,&in,t);
    assert(s.fault==SY_OK && !s.pitch_limited && s.pwm[0]<0 && s.pwm[1]>0);
    /* New HOVER entry re-centers; previous PID position does not remain the target. */
    in.requested=SY_DESCEND;tick(&s,&c,&in,12210);
    assert(s.pwm[0]==-.5f && s.pwm[1]==-.5f && !s.hover_position_ready);
    in.requested=SY_HOVER;tick(&s,&c,&in,12220);
    assert(!s.hover_position_ready && s.pwm[0]==-.5f);
    /* Interior target crossing stops, even when no sample lands in tolerance. */
    Sy_Init(&s);s.armed=true;in.requested=SY_HOVER;in.stick=0;
    seed(&s,&c,&in,0,516,13000);seed(&s,&c,&in,1,529,13000);tick(&s,&c,&in,13000);
    assert(s.pwm[0]==-.5f);
    seed(&s,&c,&in,0,560,13010);tick(&s,&c,&in,13010);
    assert(s.position_complete[0] && s.pwm[0]==0 && s.hover_position_ready);
    /* Reversed ADC wiring also enforces directional soft limits. */
    c.axis[0].cal.adc[0]=4095;c.axis[0].cal.adc[1]=522;c.axis[0].cal.adc[2]=0;
    seed(&s,&c,&in,0,4095,13020);in.stick=1;
    s.pitch_seeded=true;s.pitch_target=5;s.pitch_rate_ready=true;s.last_imu=13010;s.last_pitch=0;
    tick(&s,&c,&in,13020);assert(s.pitch_limited && s.fault==SY_OK);
    /* Full two-axis cycle with different modeled axis speeds and real calibration. */
    c=live_cfg;c.enable=true;Sy_Init(&s);s.armed=true;
    in=(SyInput){.radio_ok=true,.adc_ok=true,.imu_ok=true};
    float mm[2]={29.24f,29.24f};unsigned now=0;
    SyMode modes[]={SY_ASCEND,SY_HOVER,SY_DESCEND,SY_HOVER};
    for(unsigned n=0;n<4;n++) {
        in.requested=modes[n];bool done=false;
        for(unsigned k=0;k<4000;k++) {
            for(unsigned i=0;i<2;i++)in.adc[i]=raw_for(&c.axis[i],mm[i]);
            tick(&s,&c,&in,now);now+=10;
            assert(s.fault==SY_OK);assert(fabsf(s.pwm[0])<=.5f && fabsf(s.pwm[1])<=.5f);
            for(unsigned i=0;i<2;i++) {
                assert(s.pwm[i]==0 || fabsf(s.pwm[i])==.5f);
                mm[i]+=s.pwm[i]*c.axis[i].motor_sign*(i?2.5f:4.0f)*.01f;
                if(mm[i]<0)mm[i]=0;
                if(mm[i]>58.48f)mm[i]=58.48f;
            }
            if(s.position_complete[0] && s.position_complete[1]) {done=true;break;}
        }
        assert(done);
    }
    in.radio_ok=false;tick(&s,&c,&in,now);assert(s.fault==SY_RADIO && !s.pwm[0] && !s.pwm[1]);
    puts("PASS fixed50 directions, independent arrival, no noise restart, center-before-pitch, variable PID, directional ADC soft limit/inward recovery, reversed ADC, target crossing, asymmetric-speed UP/HOVER/DOWN/HOVER, radio stop");
    return 0;
}
