#include "position_control.h"
#include <math.h>
#include <string.h>

static float clamp(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t t;
    if (a>b) { t=a;a=b;b=t; }
    if (b>c) { t=b;b=c;c=t; }
    if (a>b) b=a;
    return b;
}

static PositionStatus fail(PositionControl *control, PositionStatus status)
{
    control->pwm_percent = 0;
    control->integral = 0;
    control->status = status;
    return status;
}

bool PositionConfig_Valid(const PositionConfig *c)
{
    if (c == NULL || !SliderCalibration_Valid(&c->calibration)) return false;
    const float values[] = {c->safe_min_mm,c->safe_max_mm,c->kp,c->ki,
        c->max_pwm_percent,c->tolerance_mm,c->target_rate_mm_s,c->max_sensor_speed_mm_s};
    for (unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i) if (!isfinite(values[i])) return false;
    float margin=c->allow_adc_endpoints ? 0.0f : 0.5f;
    if (c->allow_adc_endpoints && (c->adc_tolerance<1 || c->adc_tolerance>32)) return false;
    if (c->continuous_drive && (!c->allow_adc_endpoints || c->pulse_drive)) return false;
    if (c->pulse_drive && (!c->allow_adc_endpoints || !isfinite(c->pulse_near_mm) ||
        c->pulse_near_mm <= c->tolerance_mm || c->pulse_near_mm > 5.0f ||
        c->pulse_on_ms < 20 || c->pulse_on_ms > 100 ||
        c->pulse_off_ms < 100 || c->pulse_off_ms > 500)) return false;
    return (c->motor_sign == 1 || c->motor_sign == -1) &&
        c->safe_min_mm >= c->calibration.mm[0] + margin &&
        c->safe_max_mm <= c->calibration.mm[2] - margin &&
        c->safe_max_mm - c->safe_min_mm > 2*c->tolerance_mm &&
        c->kp > 0 && c->ki >= 0 && c->max_pwm_percent >= 1 && c->max_pwm_percent <= 100 &&
        c->tolerance_mm > 0 && c->tolerance_mm <= 1 &&
        c->target_rate_mm_s > 0 && c->max_sensor_speed_mm_s >= c->target_rate_mm_s &&
        c->stall_ms >= 500 && c->stall_ms <= 10000 &&
        c->max_move_ms >= 1000 && c->max_move_ms <= 120000 &&
        c->settle_ms >= 100 && c->settle_ms <= 2000;
}

PositionStatus Position_Start(PositionControl *s, const PositionConfig *c,
                              float target, uint16_t raw, uint32_t now)
{
    memset(s, 0, sizeof(*s));
    if (!PositionConfig_Valid(c)) return fail(s, POSITION_CONFIG_ERROR);
    if (!isfinite(target) || target < c->safe_min_mm || target > c->safe_max_mm)
        return fail(s, POSITION_TARGET_ERROR);
    if (!SliderCalibration_Position(&c->calibration, raw, &s->measured_mm))
        return fail(s, POSITION_SENSOR_ERROR);
    if (s->measured_mm < c->safe_min_mm || s->measured_mm > c->safe_max_mm)
        return fail(s, POSITION_LIMIT_ERROR);
    s->feedback_adc = s->adc_history[0] = raw;
    s->adc_count = 1;
    s->adc_index = 1;
    s->status = POSITION_RUNNING;
    s->goal_mm = target;
    if (c->allow_adc_endpoints) {
        unsigned i=target<=c->calibration.mm[1] ? 0 : 1;
        float fraction=(target-c->calibration.mm[i])/(c->calibration.mm[i+1]-c->calibration.mm[i]);
        s->goal_adc=(uint16_t)lroundf(c->calibration.adc[i]+fraction*((int)c->calibration.adc[i+1]-c->calibration.adc[i]));
    }
    s->ramp_mm = s->previous_mm = s->measured_mm;
    SignalGuard_Seed(&s->signal, raw, s->measured_mm, now);
    s->started_ms = s->last_ms = now;
    return s->status;
}

PositionStatus Position_Update(PositionControl *s, const PositionConfig *c,
                               bool sensor_ok, uint16_t raw, uint32_t now)
{
    if (s->status != POSITION_RUNNING) return s->status;
    if (!PositionConfig_Valid(c)) return fail(s, POSITION_CONFIG_ERROR);
    uint32_t elapsed = now - s->last_ms;
    if (elapsed == 0 || elapsed > 50) return fail(s, POSITION_TIMING_ERROR);
    if ((uint32_t)(now-s->started_ms) >= c->max_move_ms) return fail(s, POSITION_TIMEOUT_ERROR);
    float observed_mm;
    if (c->noise_tolerant) {
        bool valid=sensor_ok && SliderCalibration_Position(&c->calibration,raw,&observed_mm);
        if (valid) valid=observed_mm>=c->safe_min_mm && observed_mm<=c->safe_max_mm;
        SignalResult quality=SignalGuard_Update(&s->signal, &c->calibration,
            valid, raw, valid?observed_mm:0, now, c->max_sensor_speed_mm_s);
        if (quality!=SIGNAL_VALID) {
            s->last_ms=now;
            if (quality!=SIGNAL_HOLD || !valid) s->pwm_percent=0;
            if (quality==SIGNAL_FAILED) return fail(s,valid?POSITION_JUMP_ERROR:POSITION_SENSOR_ERROR);
            return s->status;
        }
    }
    if (!sensor_ok || !SliderCalibration_Position(&c->calibration, raw, &observed_mm))
        return fail(s, POSITION_SENSOR_ERROR);
    /* Validate the current observation before filtering: never mask ADC
     * failures or out-of-range positions with older, apparently good samples. */
    if (observed_mm < c->safe_min_mm || observed_mm > c->safe_max_mm)
        return fail(s, POSITION_LIMIT_ERROR);
    float dt = elapsed * 0.001f;
    bool rail_target = c->allow_adc_endpoints &&
        (s->goal_adc==c->calibration.adc[0] || s->goal_adc==c->calibration.adc[2]);
    if (c->median_adc && rail_target && raw==s->goal_adc) {
        /* First raw rail observation stops output immediately. A large jump
         * still latches a fault instead of claiming successful motion. */
        s->measured_mm=observed_mm;s->feedback_adc=raw;
        if (!c->noise_tolerant && fabsf(observed_mm-s->previous_mm)>0.25f+c->max_sensor_speed_mm_s*dt)
            return fail(s,POSITION_JUMP_ERROR);
        return fail(s,POSITION_DONE);
    }
    s->feedback_adc=raw;
    if (c->median_adc) {
        bool warming=s->adc_count<3;
        s->adc_history[s->adc_index]=raw;
        s->adc_index=(s->adc_index+1)%3;
        if (s->adc_count<3) ++s->adc_count;
        if (s->adc_count==3) s->feedback_adc=median3(s->adc_history[0],s->adc_history[1],s->adc_history[2]);
        if (warming) {
            /* Acquire three real samples with output off. Do not duplicate
             * the first reading, which could itself be an isolated spike. */
            s->pwm_percent=0;s->integral=0;s->last_ms=now;
            if (s->adc_count==3) {
                (void)SliderCalibration_Position(&c->calibration,s->feedback_adc,&s->measured_mm);
                s->previous_mm=s->ramp_mm=s->measured_mm;
            }
            return s->status;
        }
    }
    (void)SliderCalibration_Position(&c->calibration,s->feedback_adc,&s->measured_mm);
    if (!c->noise_tolerant && fabsf(s->measured_mm-s->previous_mm) > 0.25f + c->max_sensor_speed_mm_s*dt)
        return fail(s, POSITION_JUMP_ERROR);
    float previous_error = s->goal_mm-s->previous_mm;
    s->last_ms = now;
    s->previous_mm = s->measured_mm;
    s->ramp_mm += clamp(s->goal_mm-s->ramp_mm,-c->target_rate_mm_s*dt,c->target_rate_mm_s*dt);

    bool reached=fabsf(s->goal_mm-s->measured_mm) <= c->tolerance_mm;
    if (c->allow_adc_endpoints) {
        bool endpoint=s->goal_adc==c->calibration.adc[0] || s->goal_adc==c->calibration.adc[2];
        int delta=(int)s->feedback_adc-(int)s->goal_adc;
        reached=endpoint ? delta==0 : (delta>=-(int)c->adc_tolerance && delta<=(int)c->adc_tolerance);
        /* First rail observation completes the move. Never push further into
         * the ADC plateau or restart on noise after this endpoint completion. */
        if (endpoint && reached) return fail(s,POSITION_DONE);
    }
    /* Continuous bench motion completes on the first accepted arrival.
     * A sample can cross a narrow target band without landing in it; stop
     * instead of reversing repeatedly. DONE stays latched until a new move.
     * Validation and jump checks above still apply. Post-stop drift must be
     * measured by the bench script; DONE does not imply zero physical drift. */
    if (c->continuous_drive && (reached ||
        (!rail_target && previous_error*(s->goal_mm-s->measured_mm) < 0)))
        return fail(s,POSITION_DONE);
    if (reached) {
        s->pwm_percent = 0;
        s->integral = 0;
        s->pulsing = false;
        s->watching = false;
        if (!s->settling) {s->settling=true;s->settled_ms=now;}
        if ((uint32_t)(now-s->settled_ms) >= c->settle_ms) return fail(s,POSITION_DONE);
        return s->status;
    }
    s->settling = false;
    if (c->continuous_drive) {
        s->integral=0;
        s->pulsing=false;
        s->pwm_percent=(s->goal_mm>s->measured_mm ? 1 : -1)*c->motor_sign*
            (int)lroundf(c->max_pwm_percent);
    } else if (c->pulse_drive) {
        /* Full on-strength, with reduced on-time near the final goal. This
         * bypasses the PI ramp only in the opted-in fixture profile. Sensor
         * validation, settle, net-progress and timeout still run every tick.
         * The hardware writer retains its 100ms restart/reversal dead time;
         * pulse_off_ms must accommodate it. No HAL_Delay or busy waiting. */
        float error = s->goal_mm-s->measured_mm;
        bool on = true;
        s->integral=0;
        if (fabsf(error) <= c->pulse_near_mm) {
            if (!s->pulsing) {s->pulsing=true;s->pulse_started_ms=now;}
            uint32_t phase=(uint32_t)(now-s->pulse_started_ms) %
                (c->pulse_on_ms+c->pulse_off_ms);
            on=phase<c->pulse_on_ms;
        } else s->pulsing=false;
        s->pwm_percent=on ? (error>0 ? 1 : -1)*c->motor_sign*
            (int)lroundf(c->max_pwm_percent) : 0;
    } else {
        float error = s->ramp_mm-s->measured_mm;
        float trial = clamp(s->integral+c->ki*error*dt,-c->max_pwm_percent,c->max_pwm_percent);
        float output = c->kp*error + trial;
        if (fabsf(output) <= c->max_pwm_percent || output*error < 0) s->integral=trial;
        output = clamp(c->kp*error+s->integral,-c->max_pwm_percent,c->max_pwm_percent);
        s->pwm_percent = (int)lroundf(output)*c->motor_sign;
    }
    /* Only net progress toward the final target refreshes this watchdog.
     * Oscillation/noise must not keep a mechanically stalled motor powered.
     * Once watching, a zero-rounded PWM sample does not erase the timer. */
    float remaining=fabsf(s->goal_mm-s->measured_mm);
    if (!s->watching && s->pwm_percent != 0) {
        s->watching=true;s->best_error_mm=remaining;s->watch_ms=now;
    } else if (s->watching) {
        float progress=0.15f+(c->noise_tolerant?SignalNoiseMm(&c->calibration,s->feedback_adc):0);
        if (s->best_error_mm-remaining >= progress) {
            s->best_error_mm=remaining;s->watch_ms=now;
        } else if ((uint32_t)(now-s->watch_ms) >= c->stall_ms) {
            return fail(s,POSITION_STALL_ERROR);
        }
    }
    return s->status;
}

const char *PositionStatus_Name(PositionStatus s)
{
    switch(s) {
    case POSITION_IDLE:return "IDLE";
    case POSITION_RUNNING:return "RUNNING";
    case POSITION_DONE:return "DONE";
    case POSITION_CONFIG_ERROR:return "CALIBRATION_OR_CONFIG";
    case POSITION_TARGET_ERROR:return "TARGET_RANGE";
    case POSITION_SENSOR_ERROR:return "SENSOR";
    case POSITION_LIMIT_ERROR:return "POSITION_LIMIT";
    case POSITION_TIMING_ERROR:return "CONTROL_TIMING";
    case POSITION_STALL_ERROR:return "NO_MOVEMENT";
    case POSITION_TIMEOUT_ERROR:return "MOVE_TIMEOUT";
    case POSITION_JUMP_ERROR:return "SENSOR_JUMP";
    default:return "UNKNOWN";
    }
}
