#include "syringe_control.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
static float clamp(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }
static bool positive(float x) { return isfinite(x)&&x>0; }
static bool nonnegative(float x) { return isfinite(x)&&x>=0; }
static float upper(const SyAxis *a) { return a->guard_mm ? a->guard_mm : a->cal.mm[2]; }
static bool position(const SyAxis *a,uint16_t raw,float *mm) {
    if (SliderCalibration_Position(&a->cal,raw,mm)) return true;
    if (!a->guard_mm) return false;
    float f=((int)raw-(int)a->cal.adc[2])/(float)((int)a->guard_adc-(int)a->cal.adc[2]);
    if (!(f>=0 && f<=1)) return false;
    *mm=a->cal.mm[2]+f*(a->guard_mm-a->cal.mm[2]);return true;
}
static float wrap(float x) { while(x>180)x-=360; while(x< -180)x+=360; return x; }
bool Sy_Validate(const SyConfig *c) {
    if (!positive(c->pwm_limit)||c->pwm_limit>1.0f) return false;
    if (!c->fixed_position && (!positive(c->position_kp)||!nonnegative(c->position_ki)||
        !positive(c->position_rate_mm_s))) return false;
    for(unsigned i=0;i<2;i++) {
        const SyAxis *a=&c->axis[i];
        if(!SliderCalibration_Valid(&a->cal)||a->cal.mm[0]!=0||a->cal.mm[2]<a->stroke_mm||!positive(a->stroke_mm)||a->stroke_mm<3||
           !positive(a->volume_span_cc)||(a->motor_sign!=1&&a->motor_sign!=-1)) return false;
        if (a->guard_mm && (!isfinite(a->guard_mm)||a->guard_mm<=a->cal.mm[2]||
            a->guard_adc<10||a->guard_adc>4085||
            ((int)a->guard_adc-a->cal.adc[2])*(a->cal.adc[2]>a->cal.adc[1]?1:-1)<5)) return false;
        for(unsigned m=0;m<3;m++) {
            float x=c->base_cc[m][i]*a->stroke_mm/a->volume_span_cc;
            /* Inclusive calibrated endpoints; reject targets outside travel. */
            if(!isfinite(x)||x<0||x>a->stroke_mm||x>upper(a)) return false;
        }
    }
    return !c->angle_enable || (positive(c->pitch_kp)&&nonnegative(c->pitch_ki)&&
        nonnegative(c->pitch_kd)&&positive(c->max_angle_deg)&&c->max_angle_deg<=15&&
        positive(c->angle_rate_deg_s)&&positive(c->pitch_stop_deg)&&
        positive(c->pitch_resume_deg)&&c->pitch_resume_deg>c->pitch_stop_deg&&
        positive(c->pitch_stop_rate)&&positive(c->pitch_resume_rate)&&c->pitch_resume_rate>c->pitch_stop_rate&&
        (c->pitch_sign==1||c->pitch_sign==-1));
}
void Sy_Init(SyControl *s) { memset(s,0,sizeof(*s)); }
void Sy_Fail(SyControl *s,SyFault fault) {
    s->recovery_wait=false;
    if(s->mode!=SY_FAULT) {
        s->blocked_switch=s->last_requested;s->recovery_wait=false;
        /* Reacquire feedback while stopped, including changed sensor baselines. */
        memset(s->signal,0,sizeof s->signal);
    }
    s->mode=SY_FAULT; s->fault=fault; s->armed=false;
    s->pwm[0]=s->pwm[1]=0; s->integral[0]=s->integral[1]=0;
    s->pitch_integral=0; s->pitch_pwm=0; s->pitch_holding=false;
}
/* This detects lack of slider movement, not motor torque or actual water flow. */
static void WatchMotion(SyControl *s,const SyConfig *c,uint32_t now) {
    for(unsigned i=0;i<2;i++) {
        if(fabsf(s->pwm[i])<0.01f) s->watching[i]=false;
        else if(!s->watching[i]||fabsf(s->position[i]-s->watch_position[i])>=
            0.15f+SignalNoiseMm(&c->axis[i].cal,s->signal[i].raw)) {
            s->watching[i]=true;s->watch_position[i]=s->position[i];s->stall_since[i]=now;
        } else if((uint32_t)(now-s->stall_since[i])>=DEMO_STALL_MS) {Sy_Fail(s,SY_STALL);return;}
    }
}
/* No blocking delay: each axis stops independently on arrival/crossing.
 * Completion is held until the switch mode changes, including rail noise. */
static void FixedPosition(SyControl *s,const SyConfig *c,unsigned m,uint32_t now) {
    for(unsigned i=0;i<2;i++) {
        float goal=c->base_cc[m][i]*c->axis[i].stroke_mm/c->axis[i].volume_span_cc;
        s->target[i]=goal;s->integral[i]=0;
        float error=goal-s->position[i];
        bool endpoint=goal<=0 || goal>=c->axis[i].stroke_mm;
        bool arrived=(goal<=0 && s->position[i]<=0) ||
            (goal>=c->axis[i].stroke_mm && s->position[i]>=goal) ||
            (!endpoint && (fabsf(error)<=0.25f ||
             (s->move_direction[i]>0 && error<=0) ||
             (s->move_direction[i]<0 && error>=0)));
        if(arrived)s->position_complete[i]=true;
        if(s->position_complete[i]) {s->watching[i]=false;continue;}
        if(!s->move_direction[i])s->move_direction[i]=error>0?1:-1;
        s->pwm[i]=s->move_direction[i]*c->pwm_limit*c->axis[i].motor_sign;
    }
    if(s->mode==SY_HOVER && s->position_complete[0] && s->position_complete[1]) {
        s->hover_position_ready=true;s->pitch_seeded=false;
        s->action_since=now; /* angle stage has its own motion timeout */
    }
    WatchMotion(s,c,now);
}
/* Accepted slider ADC, supporting either potentiometer wiring direction. */
static bool OutwardAtLimit(const SyControl *s,const SyConfig *c,unsigned i,float motion) {
    const SyAxis *a=&c->axis[i];int raw=s->signal[i].raw;
    bool increasing=a->cal.adc[2]>a->cal.adc[0];
    bool low=increasing?raw<=a->cal.adc[0]:raw>=a->cal.adc[0];
    bool high=increasing?raw>=a->cal.adc[2]:raw<=a->cal.adc[2];
    return (motion<0 && low) ||
        (motion>0 && (high || s->position[i]>=a->stroke_mm));
}
void Sy_Tick(SyControl *s,const SyConfig *c,const SyInput *in,uint32_t now,float dt) {
    s->pwm[0]=s->pwm[1]=0;
    s->pitch_limited=false;
    bool recovering=s->mode==SY_FAULT;
    if(!c->enable) { s->mode=SY_OFF; s->armed=false; return; }
    if(!Sy_Validate(c)) { Sy_Fail(s,SY_CONFIG); return; }
    if(!positive(dt)||dt>0.05f) { Sy_Fail(s,SY_TIMING); return; }
    bool radio=in->radio_ok&&(uint32_t)(now-in->radio_ms)<=300&&
        isfinite(in->stick)&&fabsf(in->stick)<=1&&in->requested>=SY_ASCEND&&in->requested<=SY_DESCEND;
    if(recovering) {
        bool healthy=radio && in->adc_ok;
        for(unsigned i=0;i<2;i++) {
            float mm=0;
            bool ok=in->adc_ok && position(&c->axis[i],in->adc[i],&mm);
            if(SignalGuard_Update(&s->signal[i],&c->axis[i].cal,ok,in->adc[i],mm,now,15)!=SIGNAL_VALID) healthy=false;
        }
        if(in->requested==SY_HOVER && c->angle_enable &&
           (!in->imu_ok || !isfinite(in->pitch_deg) || fabsf(in->pitch_deg)>180 || now-in->imu_ms>100)) healthy=false;
        /* A verified radio recovery may resume the held switch. Other fault
         * reasons still require a changed switch; CH4 alone is not a retry. */
        bool same_switch_allowed=c->radio_auto_resume && s->fault==SY_RADIO;
        if(!healthy || (!same_switch_allowed && in->requested==s->blocked_switch)) {
            s->recovery_wait=false;return;
        }
        if(!s->recovery_wait || s->retry_switch!=in->requested) {
            s->recovery_wait=true;s->retry_switch=in->requested;s->recovery_since=now;return;
        }
        if(now-s->recovery_since<DEMO_RECOVERY_MS) return;
        Sy_Init(s);s->armed=true;s->action_since=now;
    }
    if(radio) s->last_requested=in->requested;
    if(!radio) {
        if(s->armed) Sy_Fail(s,SY_RADIO);
        else {s->mode=SY_OFF;s->hover_wait=false;}
        return;
    }
    bool waiting=false;
    for(unsigned i=0;i<2;i++) {
        float mm=0;const SyAxis *a=&c->axis[i];
        bool ok=in->adc_ok && position(a,in->adc[i],&mm);
        SignalResult q=SignalGuard_Update(&s->signal[i],&a->cal,ok,in->adc[i],mm,now,15);
        if(q==SIGNAL_FAILED) {Sy_Fail(s,SY_SENSOR);return;}
        if(q!=SIGNAL_VALID) {waiting=true;continue;}
        s->position[i]=mm;
        /* ADC rails alone are not sensor failures. Motion bounds are separate. */
    }
    if(waiting) return; /* brief hold, not a latched error; next good samples resume */
    if(!s->armed) {
        if(in->requested!=SY_HOVER || fabsf(in->stick)>0.05f) {s->hover_wait=false;return;}
        if(!s->hover_wait) {s->hover_wait=true;s->hover_since=now;}
        if((uint32_t)(now-s->hover_since)<500) return;
        s->armed=true;s->target[0]=s->position[0];s->target[1]=s->position[1];
    }
    SyMode previous=s->mode; s->mode=in->requested;
    if(previous!=s->mode) {
        s->hover_position_ready=false;s->pitch_seeded=false;
        s->pitch_integral=0;s->pitch_holding=false;
        for(unsigned i=0;i<2;i++) {
            s->position_complete[i]=false;s->move_direction[i]=0;
            s->watching[i]=false;s->integral[i]=0;
        }
    }
    bool angle=s->mode==SY_HOVER&&c->angle_enable&&
        (!c->fixed_position || s->hover_position_ready);
    if(angle && (!in->imu_ok||!isfinite(in->pitch_deg)||fabsf(in->pitch_deg)>180||
        (uint32_t)(now-in->imu_ms)>100)) {Sy_Fail(s,SY_IMU);return;}
    if(previous!=s->mode || (!s->watching[0] && !s->watching[1])) s->action_since=now;
    if(now-s->action_since>=60000 && (s->watching[0]||s->watching[1])) {
        Sy_Fail(s,SY_TIMEOUT);return;
    }
    float desired[2], fraction=1;
    unsigned m=(unsigned)s->mode-(unsigned)SY_ASCEND;
    s->pitch_pwm=0;
    if(c->fixed_position && !angle) {
        FixedPosition(s,c,m,now);return;
    }
    if(angle) {
        /* Fixed-position mode enters only after BOTH HOVER targets arrived.
         * Then angle error requests opposing PWM; do not re-center each tick. */
        if(previous!=SY_HOVER||!s->pitch_seeded) {
            s->pitch_target=in->pitch_deg; s->last_pitch=in->pitch_deg;
            s->last_imu=in->imu_ms;s->pitch_rate=0;s->pitch_integral=0;
            s->pitch_seeded=true;s->pitch_holding=false;s->pitch_rate_ready=false;
            for(unsigned i=0;i<2;i++) {s->integral[i]=0;s->watching[i]=false;}
        }
        if(in->imu_ms!=s->last_imu) {
            float sample_dt=(uint32_t)(in->imu_ms-s->last_imu)*0.001f;
            if(sample_dt>0 && sample_dt<=0.1f) {
                float rate=wrap(in->pitch_deg-s->last_pitch)/sample_dt;
                s->pitch_rate+=(sample_dt/(0.08f+sample_dt))*(rate-s->pitch_rate);
                s->pitch_rate_ready=true;
            } else s->pitch_rate_ready=false;
            s->last_pitch=in->pitch_deg;s->last_imu=in->imu_ms;
        }
        float requested=in->stick*c->max_angle_deg;
        s->pitch_target+=clamp(requested-s->pitch_target,-c->angle_rate_deg_s*dt,c->angle_rate_deg_s*dt);
        float command_error=wrap(requested-in->pitch_deg);
        float error=wrap(s->pitch_target-in->pitch_deg);
        /* Discard old position targets every tick. Reaching the angle always
         * overrides any leftover piston error, including after UP/DOWN. */
        for(unsigned i=0;i<2;i++) s->target[i]=s->position[i];
        if(!s->pitch_rate_ready) return;
        if(s->pitch_holding && (fabsf(command_error)>=c->pitch_resume_deg ||
                               fabsf(s->pitch_rate)>=c->pitch_resume_rate)) {
            s->pitch_holding=false;s->pitch_integral=0;
        }
        if(!s->pitch_holding && fabsf(command_error)<=c->pitch_stop_deg &&
           fabsf(s->pitch_rate)<=c->pitch_stop_rate) s->pitch_holding=true;
        if(s->pitch_holding) {
            s->pitch_integral=0;
            for(unsigned i=0;i<2;i++) {s->integral[i]=0;s->watching[i]=false;}
            return; /* both PWM already zero; no position loop runs in HOVER */
        }
        float trial=clamp(s->pitch_integral+c->pitch_ki*error*dt,-c->pwm_limit,c->pwm_limit);
        float u=c->pitch_kp*error+trial-c->pitch_kd*s->pitch_rate;
        if(fabsf(u)<=c->pwm_limit || (u>c->pwm_limit&&error<0) || (u< -c->pwm_limit&&error>0)) s->pitch_integral=trial;
        s->pitch_pwm=clamp(c->pitch_kp*error+s->pitch_integral-c->pitch_kd*s->pitch_rate,
                           -c->pwm_limit,c->pwm_limit)*c->pitch_sign;
        /* Check BOTH directions before assigning either output. A travel
         * limit is a directional hold, not a latched fault. Inward motion can
         * resume on the next valid PID request without a reset/re-arm. */
        for(unsigned i=0;i<2;i++) {
            float motion=(i==0?s->pitch_pwm:-s->pitch_pwm);
            if(OutwardAtLimit(s,c,i,motion)) {
                s->pitch_limited=true;s->pitch_integral=0;s->pitch_pwm=0;
                s->pwm[0]=s->pwm[1]=0;
                s->watching[0]=s->watching[1]=false;
                return;
            }
        }
        for(unsigned i=0;i<2;i++)
            s->pwm[i]=(i==0?s->pitch_pwm:-s->pitch_pwm)*c->axis[i].motor_sign;
        WatchMotion(s,c,now);
        return;
    }
    s->pitch_seeded=false;s->pitch_holding=false;s->pitch_rate_ready=false;s->pitch_integral=0;
    if(previous!=s->mode) for(unsigned i=0;i<2;i++) {
        s->target[i]=s->position[i];s->integral[i]=0;s->watching[i]=false;s->endpoint_reached[i]=false;
    }
    for(unsigned i=0;i<2;i++) {
        float v=c->base_cc[m][i];
        desired[i]=v*c->axis[i].stroke_mm/c->axis[i].volume_span_cc;
        float distance=fabsf(desired[i]-s->target[i]);
        if(distance>0) fraction=fminf(fraction,c->position_rate_mm_s*dt/distance);
    }
    for(unsigned i=0;i<2;i++) {
        s->target[i]+=(desired[i]-s->target[i])*fraction;
        float error=s->target[i]-s->position[i];
        bool low_end=desired[i]<=0;
        bool high_end=desired[i]>=c->axis[i].stroke_mm;
        /* Endpoints stop on accepted feedback, not the interior 0.25 mm band.
         * Latch each axis independently so rail noise cannot repeatedly drive
         * a held UP/DOWN command. A new mode permits the next movement. */
        if((low_end && s->position[i]<=0) ||
           (high_end && s->position[i]>=c->axis[i].stroke_mm))
            s->endpoint_reached[i]=true;
        if((low_end||high_end) && s->endpoint_reached[i]) {
            s->integral[i]=0;s->watching[i]=false;continue;
        }
        bool final_endpoint=(low_end||high_end) && s->target[i]==desired[i];
        if(!final_endpoint && fabsf(error)<=0.25f) {
            s->integral[i]=0;s->watching[i]=false;continue;
        }
        if((error<0 && s->position[i]<=0) || (error>0 && s->position[i]>=upper(&c->axis[i]))) {
            Sy_Fail(s,SY_LIMIT);return;
        }
        float trial=clamp(s->integral[i]+c->position_ki*error*dt,-c->pwm_limit,c->pwm_limit);
        float u=c->position_kp*error+trial;
        if(fabsf(u)<=c->pwm_limit||(u>c->pwm_limit&&error<0)||(u< -c->pwm_limit&&error>0))s->integral[i]=trial;
        s->pwm[i]=clamp(c->position_kp*error+s->integral[i],-c->pwm_limit,c->pwm_limit)*c->axis[i].motor_sign;
    }
    WatchMotion(s,c,now);
}
