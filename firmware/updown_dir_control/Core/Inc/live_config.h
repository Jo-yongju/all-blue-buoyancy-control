/* ENDPOINT LIMITATION (2026-11-09): ADC 0/4095 represent intervals,
 * not unique positions: sensor1 ~2/4 mm, sensor2 ~2/3.5 mm.
 * Existing 3-point conversion is provisional. Do NOT enable motor_sign or
 * LIVE solely from these values. Encoder observer does not authorize travel.
 * See docs/ENCODER_OBSERVER_20261109.txt. */
#pragma once
#include "syringe_control.h"
/* Slider measurements supplied 2026-11-09: 0 / midpoint / 58.48 mm.
 * PA0: 0,522,4095; PA1: 0,529,4095. Midpoint is mapped to 29.24 mm.
 * Confirm intermediate positions and installed travel before motor control.
 * Sensor calibration does not confirm hydraulic volume, motor direction,
 * or guard travel. See docs/SLIDER_CALIBRATION_20261109.txt. */
/* MDD1/PA0: negative command fills (confirmed 2026-11-12).
 * MDD2 also uses -1 by agreed identical wiring; verify on hardware.
 * Axis0/MDD1/PA0=front, axis1/MDD2/PA1=rear. */
static const SyConfig live_cfg={
    .enable=false, .angle_enable=true, .fixed_position=true,
    .radio_auto_resume=true,
    .axis={
        {.cal={{0,522,4095},{0,29.24f,58.48f},true},.motor_sign=-1,.stroke_mm=58.48f,.volume_span_cc=120,.guard_mm=0,.guard_adc=0},
        {.cal={{0,529,4095},{0,29.24f,58.48f},true},.motor_sign=-1,.stroke_mm=58.48f,.volume_span_cc=120,.guard_mm=0,.guard_adc=0}
    },
    .base_cc={{0,0},{60,60},{120,120}}, /* per module; HOVER requires wet trim */
    /* 2026-11-16: LIVE/TEAM duty ceiling 50%; applies to position AND pitch.
     * fixed_position uses this ceiling continuously for positioning.
     * Pitch PID remains variable duty. No measured speed loop is used. */
    .position_kp=.03f,.position_ki=.005f,.pwm_limit=.5f,
    .pitch_kp=.06f,.pitch_ki=.005f,.pitch_kd=.02f,.pitch_sign=-1,
    /* Provisional bench gains: PWM/deg, PWM/(deg*s), PWM/(deg/s).
     * Stop/restart hysteresis prevents chatter; tune with real IMU + mechanics. */
    .pitch_stop_deg=.5f,.pitch_resume_deg=.8f,.pitch_stop_rate=1,.pitch_resume_rate=2,
    .max_angle_deg=5,.angle_rate_deg_s=10,.position_rate_mm_s=5
};
/* Nose-up positive, axis 0 front, increasing position adds ballast.
 * Confirmed radio mapping (2026-11-11): CH6 SwC 1000 UP,1500 HOVER,2000 DOWN;
 * CH4 LH right -> nose up (2000). Physical IMU and motor signs remain measured.
 * TEAM publisher supplies normalized nose-up-positive degrees; LIVE_PITCH_*
 * below apply ONLY to the standalone BNO driver, not TEAM input. */
#define LIVE_REVERSE_SWITCH 0
#define LIVE_REVERSE_CH4 0
#define LIVE_PITCH_SIGN 1.0f
#define LIVE_LEVEL_DEG 0.0f
