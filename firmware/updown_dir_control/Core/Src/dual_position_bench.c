#include "dual_position_bench.h"
#include "position_config.h"
#include <string.h>

void DualPosition_Init(DualPositionBench *s)
{
    memset(s,0,sizeof *s);
    for (unsigned i=0;i<2;i++) {
        PositionConfig *c=&s->config[i];
        *c=position_config[i];
        /* Match the verified single-motor BENCH1/2 arrival controller. */
        c->motor_sign=-1; c->safe_min_mm=0; c->safe_max_mm=58.48f;
        c->allow_adc_endpoints=true; c->adc_tolerance=5; c->median_adc=true;
        c->kp=15; c->ki=5; c->max_pwm_percent=50; c->target_rate_mm_s=3;
        c->stall_ms=DEMO_STALL_MS; c->noise_tolerant=true;
        c->pulse_drive=false; c->continuous_drive=true;
    }
}
PositionStatus DualPosition_Start(DualPositionBench *s,unsigned target,
    const uint16_t raw[2],uint32_t now)
{
    s->fault_axis=0;
    if (target>2) return POSITION_TARGET_ERROR;
    for (unsigned i=0;i<2;i++) {
        PositionStatus r=Position_Start(&s->axis[i],&s->config[i],
            s->config[i].calibration.mm[target],raw[i],now);
        if (r!=POSITION_RUNNING) {s->fault_axis=i+1;return r;}
    }
    return POSITION_RUNNING;
}
PositionStatus DualPosition_Update(DualPositionBench *s,const bool ok[2],
    const uint16_t raw[2],uint32_t now)
{
    bool done=true;
    for (unsigned i=0;i<2;i++) {
        PositionStatus r=Position_Update(&s->axis[i],&s->config[i],ok[i],raw[i],now);
        if (r!=POSITION_RUNNING && r!=POSITION_DONE) {
            s->fault_axis=i+1;
            s->axis[0].pwm_percent=s->axis[1].pwm_percent=0;
            return r;
        }
        if (r!=POSITION_DONE) done=false;
    }
    return done?POSITION_DONE:POSITION_RUNNING;
}
