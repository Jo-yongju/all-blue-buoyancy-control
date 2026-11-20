#ifndef SIGNAL_QUALITY_H
#define SIGNAL_QUALITY_H
#include "slider_calibration.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* Bench measurements: stationary ADC range up to 20 counts. These are
 * provisional demo settings, not electrical fault detection guarantees. */
#define DEMO_NOISE_ADC 20U
#define DEMO_STALL_MS 8000U
#define DEMO_RECOVERY_MS 200U

typedef enum { SIGNAL_VALID, SIGNAL_HOLD, SIGNAL_WAIT, SIGNAL_FAILED } SignalResult;
typedef struct {
    bool seeded, suspect, recovering;
    uint16_t raw;
    float mm;
    uint32_t accepted_ms, suspect_ms, good_since;
} SignalGuard;

static inline float SignalNoiseMm(const SliderCalibration *c,uint16_t raw)
{
    float center=0,lo=0,hi=0;
    unsigned min=c->adc[0]<c->adc[2]?c->adc[0]:c->adc[2];
    unsigned max=c->adc[0]>c->adc[2]?c->adc[0]:c->adc[2];
    unsigned lower=raw>DEMO_NOISE_ADC?raw-DEMO_NOISE_ADC:0;
    unsigned upper=(unsigned)raw+DEMO_NOISE_ADC;
    if(lower<min)lower=min;
    if(upper>max)upper=max;
    if(!SliderCalibration_Position(c,raw,&center) ||
       !SliderCalibration_Position(c,(uint16_t)lower,&lo) ||
       !SliderCalibration_Position(c,(uint16_t)upper,&hi)) return 0;
    return fmaxf(fabsf(center-lo),fabsf(hi-center));
}
static inline void SignalGuard_Seed(SignalGuard *g,uint16_t raw,float mm,uint32_t now)
{
    memset(g,0,sizeof *g);g->seeded=true;g->raw=raw;g->mm=mm;g->accepted_ms=now;
}
static inline SignalResult SignalGuard_Update(SignalGuard *g,const SliderCalibration *c,
    bool valid,uint16_t raw,float mm,uint32_t now,float max_speed)
{
    if(!g->seeded && valid) {SignalGuard_Seed(g,raw,mm,now);return SIGNAL_VALID;}
    uint32_t dt=now-g->accepted_ms;
    if(dt>100)dt=100; /* Never allow an arbitrary step just because time passed. */
    float allowance=0.25f+max_speed*dt*.001f+SignalNoiseMm(c,g->raw);
    bool plausible=valid && g->seeded && fabsf(mm-g->mm)<=allowance;
    if(plausible) {
        if(g->suspect && now-g->suspect_ms>=30) {
            if(!g->recovering){g->recovering=true;g->good_since=now;}
            if(now-g->good_since<100)return SIGNAL_WAIT;
        }
        SignalGuard_Seed(g,raw,mm,now);return SIGNAL_VALID;
    }
    g->recovering=false;
    if(!g->suspect){g->suspect=true;g->suspect_ms=now;}
    uint32_t bad_ms=now-g->suspect_ms;
    if(bad_ms>=500)return SIGNAL_FAILED;
    /* A single numeric outlier may retain the previous command for <=30ms.
     * Read failures never retain powered output. Accepted position stays old. */
    return valid && bad_ms<30 ? SIGNAL_HOLD : SIGNAL_WAIT;
}

typedef struct {
    bool sampled,stable;
    uint16_t anchor;
    uint32_t since,last_ms;
} SignalHealth;
static inline void SignalHealth_Update(SignalHealth *h,bool ok,uint16_t raw,uint32_t now)
{
    bool continuous=h->sampled && now-h->last_ms<=30;
    h->sampled=true;h->last_ms=now;
    if(!ok){h->stable=false;return;}
    if(!h->stable || !continuous || abs((int)raw-(int)h->anchor)>(int)DEMO_NOISE_ADC) {
        h->stable=true;h->since=now;h->anchor=raw;
    }
}
static inline bool SignalHealth_Ready(const SignalHealth *h,uint32_t now)
{
    return h->sampled && h->stable && now-h->last_ms<=30 && now-h->since>=DEMO_RECOVERY_MS;
}
#endif
