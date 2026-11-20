#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H
#include "slider_calibration.h"
#include "signal_quality.h"

typedef struct {
    SliderCalibration calibration;
    /* Explicit bench opt-in. Default profiles retain the 0.5 mm end margin. */
    bool allow_adc_endpoints;
    bool median_adc;                 /* Explicit opt-in to 3-sample temporal median. */
    /* Bench-only fallback for a motor that runs at full duty but stalls at
     * reduced duty. Default/production profiles keep the PI controller. */
    bool pulse_drive;
    /* Bench continuous output: stop and latch at first arrival/crossing. */
    bool continuous_drive;
    bool noise_tolerant; /* Opt-in measured-noise allowance and bounded sample hold. */
    float pulse_near_mm;             /* Estimated distance, not verified linear travel. */
    uint32_t pulse_on_ms, pulse_off_ms;
    uint16_t adc_tolerance;           /* Interior targets only; rail targets require exact ADC. */
    int motor_sign;                  /* +1 or -1 AFTER a short direction test */
    float safe_min_mm, safe_max_mm;
    float kp, ki;                    /* PWM percent/mm, percent/(mm*s) */
    float max_pwm_percent;
    float tolerance_mm, target_rate_mm_s, max_sensor_speed_mm_s;
    uint32_t stall_ms, max_move_ms, settle_ms;
} PositionConfig;

typedef enum {
    POSITION_IDLE, POSITION_RUNNING, POSITION_DONE,
    POSITION_CONFIG_ERROR, POSITION_TARGET_ERROR, POSITION_SENSOR_ERROR,
    POSITION_LIMIT_ERROR, POSITION_TIMING_ERROR, POSITION_STALL_ERROR,
    POSITION_TIMEOUT_ERROR, POSITION_JUMP_ERROR
} PositionStatus;

typedef struct {
    PositionStatus status;
    SignalGuard signal;
    float measured_mm, goal_mm, ramp_mm, integral;
    float previous_mm, best_error_mm; /* Best target error at last progress checkpoint. */
    int pwm_percent;
    uint16_t goal_adc;
    uint16_t adc_history[3], feedback_adc;
    unsigned adc_count, adc_index;
    uint32_t started_ms, last_ms, watch_ms, settled_ms;
    bool watching, settling;
    bool pulsing;
    uint32_t pulse_started_ms;
} PositionControl;

bool PositionConfig_Valid(const PositionConfig *config);
PositionStatus Position_Start(PositionControl *control, const PositionConfig *config,
                              float target_mm, uint16_t raw, uint32_t now);
PositionStatus Position_Update(PositionControl *control, const PositionConfig *config,
                               bool sensor_ok, uint16_t raw, uint32_t now);
const char *PositionStatus_Name(PositionStatus status);
#endif
