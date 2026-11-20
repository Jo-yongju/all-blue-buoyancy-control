#include "input_trial.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

/* Synthetic feedback only. These calibration points never enable a hardware output. */
static const SyConfig fixture = {
    .enable=true, .angle_enable=true, .fixed_position=true,
    .axis={
        {.cal={{100,630,3950},{0,29.24f,60.0f},true},.motor_sign=1,.stroke_mm=58.48f,.volume_span_cc=120},
        {.cal={{3900,3350,120},{0,29.24f,60.0f},true},.motor_sign=1,.stroke_mm=58.48f,.volume_span_cc=120}
    },
    .base_cc={{20,20},{60,60},{120,120}},
    .position_kp=.03f,.position_ki=.005f,.pwm_limit=.4f,
    .pitch_kp=.06f,.pitch_ki=.005f,.pitch_kd=.02f,.pitch_sign=1,
    /* Provisional bench gains: PWM/deg, PWM/(deg*s), PWM/(deg/s).
     * Stop/restart hysteresis prevents chatter; tune with real IMU + mechanics. */
    .pitch_stop_deg=.5f,.pitch_resume_deg=.8f,.pitch_stop_rate=1,.pitch_resume_rate=2,
    .max_angle_deg=5,.angle_rate_deg_s=10,.position_rate_mm_s=10
};

void Trial_Start(InputTrial *t, TrialSource source, unsigned channel, uint32_t now)
{
    memset(t,0,sizeof *t);
    t->source=source;
    t->switch_channel=channel;
    t->tick_ms=now;
    t->input.requested=SY_HOVER;
    t->input.adc_ok=t->input.imu_ok=true;
    t->fresh_sim=true;
    for (unsigned i=0;i<2;i++) t->model_mm[i]=fixture.axis[i].cal.mm[1];
}

void Trial_ReceiverCommand(InputTrial *t,const ReceiverCommand *c)
{
    if(t->source!=TRIAL_RC) return;
    memcpy(t->channels,c->channels,sizeof t->channels);
    t->have_frame=true; t->frame_ms=c->received_ms;
    if(!c->command_valid) return;
    t->input.requested=c->requested; t->input.stick=c->stick;
    t->input.radio_ms=c->received_ms; t->input.radio_ok=true;
}
/* Host test adapter. Firmware consumes the command decoded by ReceiverTask. */
void Trial_Frame(InputTrial *t,const uint16_t channels[14],uint32_t now)
{
    ReceiverCommand c=ReceiverCommand_Decode(channels,now,t->switch_channel,false,false);
    Trial_ReceiverCommand(t,&c);
}

void Trial_Tick(InputTrial *t,uint32_t now)
{
    uint32_t elapsed=now-t->tick_ms;
    if (t->source==TRIAL_OFF || elapsed<10) return;
    t->tick_ms=now;
    if (t->source==TRIAL_SIM) {
        t->input.radio_ok=t->fresh_sim;
        if (t->fresh_sim) t->input.radio_ms=now;
    }
    t->input.imu_ms=now; /* Pitch entered by operator, held constant. */
    if (!t->fixed_adc) {
        for (unsigned i=0;i<2;i++) {
            const SliderCalibration *c=&fixture.axis[i].cal;
            float moved=t->control.pwm[i]*20.0f*(elapsed*.001f);
            float mm=fmaxf(0,fminf(c->mm[2],t->model_mm[i]+moved));
            t->model_mm[i]=mm;
            unsigned j=mm<c->mm[1] ? 0 : 1;
            float f=(mm-c->mm[j])/(c->mm[j+1]-c->mm[j]);
            t->input.adc[i]=(uint16_t)lroundf(c->adc[j]+f*((int)c->adc[j+1]-c->adc[j]));
        }
    }
    Sy_Tick(&t->control,&fixture,&t->input,now,elapsed*.001f);
}

static bool numbers(const char *text, int count, long min, long max, long *out)
{
    for (int i=0;i<count;i++) {
        if (i && !isspace((unsigned char)*text)) return false;
        char *end;
        errno=0;
        long value=strtol(text,&end,10);
        if (end==text || errno || value<min || value>max) return false;
        out[i]=value;
        text=end;
    }
    return *text=='\0';
}

bool Trial_Command(InputTrial *t,const char *line,uint32_t now,char *reply,size_t cap)
{
    long v[2];
    const char *msg="OK";
    if (!strcmp(line,"MODE SIM")) Trial_Start(t,TRIAL_SIM,0,now);
    else if (!strcmp(line,"MODE RC")) Trial_Start(t,TRIAL_RC,0,now);
    else if (!strncmp(line,"MODE RC ",8) && numbers(line+8,1,1,14,v) && v[0]!=4)
        Trial_Start(t,TRIAL_RC,(unsigned)v[0],now);
    else if (!strcmp(line,"STOP") || !strcmp(line,"!")) Trial_Start(t,TRIAL_OFF,0,now);
    else if (t->source==TRIAL_OFF) return false;
    else if (!strcmp(line,"STATUS")) {
        static const char *modes[]={"OFF","UP","HOVER","DOWN","FAULT"};
        static const char *faults[]={"OK","CONFIG","RADIO","SENSOR","IMU","LIMIT","STALL","TIMING","TIMEOUT"};
        const SyControl *s=&t->control;
        snprintf(reply,cap,"%s %s %s armed=%u target_um=%ld,%ld pos_um=%ld,%ld virtual_pwm_pct=%ld,%ld HW=OFF\r\n",
            t->source==TRIAL_SIM?"SIM":"RC",modes[s->mode],faults[s->fault],(unsigned)s->armed,
            lroundf(s->target[0]*1000),lroundf(s->target[1]*1000),lroundf(s->position[0]*1000),
            lroundf(s->position[1]*1000),lroundf(s->pwm[0]*100),lroundf(s->pwm[1]*100));
        return true;
    } else if (!strcmp(line,"PITCH STATUS")) {
        const SyControl *s=&t->control;
        snprintf(reply,cap,"PITCH control=ANGLE_PWM hold=%u rate_ok=%u target_mdeg=%ld actual_mdeg=%ld rate_mdeg_s=%ld pwm_pct=%ld,%ld HW=OFF\r\n",
            s->pitch_holding,s->pitch_rate_ready,lroundf(s->pitch_target*1000),
            lroundf(t->input.pitch_deg*1000),lroundf(s->pitch_rate*1000),
            lroundf(s->pwm[0]*100),lroundf(s->pwm[1]*100));
        return true;
    } else if (!strcmp(line,"CHANNELS") && t->source==TRIAL_RC) {
        if (!t->have_frame || now-t->frame_ms>300) msg="WAIT fresh iBUS";
        else {
            size_t used=0;
            for (unsigned i=0;i<14 && used<cap;i++) {
                int n=snprintf(reply+used,cap-used,"%u=%u%s",i+1,t->channels[i],i==13?"\r\n":" ");
                if (n<0) break;
                used+=(size_t)n;
            }
            return true;
        }
    } else if (t->source==TRIAL_SIM && !strcmp(line,"SW DOWN")) t->input.requested=SY_DESCEND;
    else if (t->source==TRIAL_SIM && !strcmp(line,"SW UP")) t->input.requested=SY_ASCEND;
    else if (t->source==TRIAL_SIM && !strcmp(line,"SW HOVER")) t->input.requested=SY_HOVER;
    else if (t->source==TRIAL_SIM && !strncmp(line,"STICK ",6) && numbers(line+6,1,-100,100,v))
        t->input.stick=v[0]*.01f;
    else if (t->source==TRIAL_SIM && !strcmp(line,"LINK OFF")) t->fresh_sim=false;
    else if (!strncmp(line,"PITCH ",6) && numbers(line+6,1,-90,90,v)) t->input.pitch_deg=(float)v[0];
    else if (!strncmp(line,"ADC ",4) && numbers(line+4,2,0,4095,v)) {
        t->fixed_adc=true;
        for (unsigned i=0;i<2;i++) t->input.adc[i]=(uint16_t)v[i];
    } else msg="ERR trial command; use STATUS or STOP";
    snprintf(reply,cap,"%s\r\n",msg);
    return true;
}
