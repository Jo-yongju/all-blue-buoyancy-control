/* ENDPOINT LIMITATION (2026-11-09): ADC 0/4095 represent intervals,
 * not unique positions: sensor1 ~2/4 mm, sensor2 ~2/3.5 mm.
 * Existing 3-point conversion is provisional. Do NOT enable motor_sign or
 * LIVE solely from these values. Encoder observer does not authorize travel.
 * See docs/ENCODER_OBSERVER_20261109.txt. */
#ifndef POSITION_CONFIG_H
#define POSITION_CONFIG_H
#include "position_control.h"

/* Measured slider points supplied 2026-11-09; midpoint mapped to 29.24 mm.
 * Confirmed means points entered, NOT installed accuracy or motor permission.
 * Direction remains unconfirmed (motor_sign=0); MOVE remains blocked.
 * Legacy L298N MOVE config; MDD10A LIVE uses live_config.h.
 * See docs/SLIDER_CALIBRATION_20261109.txt. */
static const PositionConfig position_config[2] = {
    {
        .calibration = {.adc={0,522,4095}, .mm={0,29.24f,58.48f}, .confirmed=true},
        .motor_sign=0,
        .safe_min_mm=1, .safe_max_mm=57.48f,
        .kp=8, .ki=2, .max_pwm_percent=30,
        .tolerance_mm=0.4f, .target_rate_mm_s=2, .max_sensor_speed_mm_s=15,
        .stall_ms=1500, .max_move_ms=60000, .settle_ms=200
    },
    {
        .calibration = {.adc={0,529,4095}, .mm={0,29.24f,58.48f}, .confirmed=true},
        .motor_sign=0,
        .safe_min_mm=1, .safe_max_mm=57.48f,
        .kp=8, .ki=2, .max_pwm_percent=30,
        .tolerance_mm=0.4f, .target_rate_mm_s=2, .max_sensor_speed_mm_s=15,
        .stall_ms=1500, .max_move_ms=60000, .settle_ms=200
    }
};
#endif
