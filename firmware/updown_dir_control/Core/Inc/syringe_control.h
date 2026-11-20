#ifndef SYRINGE_CONTROL_H
#define SYRINGE_CONTROL_H
#include <stdbool.h>
#include "slider_calibration.h"
#include "signal_quality.h"
#include <stdint.h>
typedef enum { SY_OFF, SY_ASCEND, SY_HOVER, SY_DESCEND, SY_FAULT } SyMode;
typedef enum { SY_OK, SY_CONFIG, SY_RADIO, SY_SENSOR, SY_IMU, SY_LIMIT, SY_STALL, SY_TIMING, SY_TIMEOUT } SyFault;
typedef struct {
    SliderCalibration cal;
    int motor_sign;
    float stroke_mm, volume_span_cc;
    /* Optional measured travel beyond the hydraulic full mark. */
    float guard_mm;
    uint16_t guard_adc;
} SyAxis;
typedef struct {
    bool enable, angle_enable;
    bool fixed_position; /* fixed duty positioning, then HOVER angle PID */
    bool radio_auto_resume; /* only SY_RADIO: fresh feedback + stable recovery */
    SyAxis axis[2];
    float base_cc[3][2]; /* ASCEND,HOVER,DESCEND; per MODULE (two syringes) */
    float position_kp, position_ki, pwm_limit;
    /* Angle PID output is signed PWM, NOT a water-volume target. */
    float pitch_kp, pitch_ki, pitch_kd, pitch_sign;
    float max_angle_deg, angle_rate_deg_s, position_rate_mm_s;
    float pitch_stop_deg, pitch_resume_deg, pitch_stop_rate, pitch_resume_rate;
} SyConfig;
typedef struct {
    bool radio_ok, adc_ok, imu_ok;
    SyMode requested;
    float stick, pitch_deg;
    uint16_t adc[2];
    uint32_t radio_ms, imu_ms;
} SyInput;
typedef struct {
    SyMode mode;
    SyMode last_requested, blocked_switch, retry_switch;
    bool recovery_wait;
    uint32_t recovery_since, action_since;
    SignalGuard signal[2];
    SyFault fault;
    bool armed, seeded, hover_wait;
    uint32_t hover_since, stall_since[2];
    bool watching[2];
    bool endpoint_reached[2]; /* held endpoint command stops until mode changes */
    bool position_complete[2], hover_position_ready, pitch_limited;
    int move_direction[2];
    float position[2], target[2], integral[2], watch_position[2], pwm[2];
    float pitch_target, pitch_integral, pitch_rate, last_pitch, pitch_pwm;
    uint32_t last_imu;
    bool pitch_seeded, pitch_holding, pitch_rate_ready;
} SyControl;
bool Sy_Validate(const SyConfig *c);
void Sy_Init(SyControl *s);
void Sy_Fail(SyControl *s, SyFault fault);
void Sy_Tick(SyControl *s, const SyConfig *c, const SyInput *in, uint32_t now, float dt);
#endif
