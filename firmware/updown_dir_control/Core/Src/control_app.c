#include "main.h"
#include "motor_test.h"
#include "motor_command.h"
#include "input_trial.h"
#include "trial_receiver.h"
#include "live_io.h"
#include "live_config.h"
#include <math.h>
#include "slider_sensor.h"
#include "position_control.h"
#include "position_config.h"
#include "dual_position_bench.h"
#include "runtime_policy.h"
#include "runtime_port.h"
#include "periodic_job.h"
#include "console_service.h"
#include "control_app.h"
#include "app_transport.h"
#include "app_inputs.h"
#include "team_link.h"
#include "app_tasks.h"
#include "encoder_service.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* MDD10A: PB6/TIM4_CH1 + PC4, PB9/TIM4_CH4 + PC7.
 * Legacy L298N: PA6/TIM3_CH1, PB0/PB1. Explicit DRIVER selects bench output.
 * ControlTask owns commands/output; TIM6 enforces the stop deadline. */
#define ARM_TIMEOUT_MS       10000U
#define LOOP_TIMEOUT_MS      100U
#define RESTART_DELAY_MS     100U
#define TIMER_COUNTER_HZ     1000000U
#define PWM_PERIOD_COUNTS    1000U

typedef enum {
    EVENT_NONE,
    EVENT_TIMED_STOP,
    EVENT_LOOP_TIMEOUT,
    EVENT_ARM_EXPIRED,
    EVENT_POSITION_TIMEOUT,
    EVENT_CONTROL_TIMEOUT
} MotorEvent;

static volatile bool initialized;
static volatile bool live_permit;
static volatile bool live_timeout;
static volatile uint32_t live_heartbeat;
static bool live_mode;
static bool team_mode; /* external publishers; no second UART/SPI owner */
static uint32_t live_epoch,live_tick,live_request;
static SyControl live_control;
static SyInput live_input;
static volatile bool running;
static volatile bool armed;
static volatile uint32_t deadline;
static volatile uint32_t last_task;
static volatile uint32_t stopped_at;
static volatile uint32_t armed_at;
static volatile MotorEvent event;
static volatile int commanded_percent;
static volatile uint32_t stop_epoch;
static volatile bool rx_abort;
static volatile bool rx_error;
static volatile uint32_t uart_error_count;

/* Position loop runs in ControlTask; TIM6/button may only revoke output permission.
 * The hardware write rechecks permission with IRQs masked, preventing restart
 * after an asynchronous STOP while an ADC conversion was in progress. */
static volatile bool position_running;
static volatile bool fault_latched;
static volatile MotorEvent last_fault_event;
static volatile uint32_t last_control_completed;
static PositionControl position;
/* Explicit one-motor profiles; neither enables two-axis LIVE implicitly. */
static bool dual_mode;
static volatile bool dual_running;
static DualPositionBench dual;
static SignalHealth dual_health[2];
static uint32_t dual_release;
static bool bench2; /* PA1: legacy L298N or explicit MDD10A channel 2. */
static bool bench1; /* Explicit MDD10A channel 1 / PA0 position fixture. */
static PositionConfig bench_config;
static SignalHealth retry_health;
static const PositionConfig *ActivePositionConfig(void)
{
    return (bench1 || bench2) ? &bench_config : &position_config[0];
}
static bool ReadPositionSensor(uint16_t *raw)
{
    if (!bench2) return SliderSensor_Read(raw);
    return SliderSensor_ReadAxis(1,raw);
}
/* 0: legacy L298N; 1/2: MDD10A. BENCH1/2 authorize matching PA0/PA1 fixtures.
 * LIVE/TEAM use both MDD10A channels independently of this bench selector. */
static unsigned jog_driver;
static volatile int applied_direction;
/* ControlTask is the only position-loop scheduler. Its release timestamp
 * must not be replaced by the later ADC completion time when MOVE starts. */
static uint32_t control_release_ms;
static PeriodicJob telemetry_job;
static bool trace_enabled;
static bool measurement_valid;
static uint16_t last_raw;
static uint32_t measurement_at;
static uint32_t previous_control_start;

/* ControlTask-owned measurements, since reset. Max observed != proven WCET. */
static struct {
    uint32_t calls, max_interval_ms, max_late_ms, missed_releases;
    uint32_t max_control_us, max_task_us, overruns;
} stats;

static void RecordMax(uint32_t *maximum, uint32_t sample)
{
    if (sample > *maximum) *maximum = sample;
}

static char command_line[64];

uint32_t MotorTest_Generation(void) { return stop_epoch; }

void MotorTest_Stop(void)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    ++stop_epoch;
    live_permit=false;
    live_timeout=false;
    LiveIO_Off();
    /* Clock checks also allow this function to be called during startup faults. */
    if (RCC->APB1ENR1 & RCC_APB1ENR1_TIM3EN) {
        TIM3->CCR1 = 0;
        TIM3->CCER &= ~TIM_CCER_CC1E;
    }
    if (RCC->AHB2ENR & RCC_AHB2ENR_GPIOBEN) {
        GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16;
    }
    running = false;
    position_running = false;
    dual_running = false;
    applied_direction = 0;
    armed = false;
    commanded_percent = 0;
    stopped_at = HAL_GetTick();
    if (!interrupt_mask) __enable_irq();
}

void MotorTest_CommunicationStop(bool error)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    MotorTest_Stop();
    rx_abort = true;
    if (error) { rx_error = true; ++uart_error_count; }
    if (!mask) __enable_irq();
}

void MotorTest_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM3_RELEASE_RESET();
    MotorTest_Stop();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);

    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) {
        timer_hz *= 2;
    }
    if (timer_hz % TIMER_COUNTER_HZ != 0) {
        Error_Handler();
    }

    /* 1 MHz counter / 1000 counts = 1 kHz PWM. */
    TIM3->PSC = timer_hz / TIMER_COUNTER_HZ - 1;
    TIM3->ARR = PWM_PERIOD_COUNTS - 1;
    TIM3->CCR1 = 0;
    TIM3->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->CR1 = TIM_CR1_CEN;
    __HAL_DBGMCU_FREEZE_TIM3();

    RuntimePort_InitClock();
    last_task = HAL_GetTick();
    PeriodicJob_Start(&telemetry_job, last_task, TELEMETRY_PERIOD_MS);
    initialized = true;
}

void MotorTest_Tick(uint32_t now)
{
    if (!initialized) {
        return;
    }
    if (live_permit && now-live_heartbeat>40U) {
        LiveIO_Off();
        live_timeout=true; /* Keep radio service alive for switch recovery. */
    }

    bool time_elapsed = (int32_t)(now - deadline) >= 0;
    bool loop_stalled = (uint32_t)(now - last_task) > LOOP_TIMEOUT_MS;
    bool control_stalled = (position_running || dual_running) &&
        (uint32_t)(now - last_control_completed) > 2U * CONTROL_PERIOD_MS;
    if (running && (time_elapsed || loop_stalled || control_stalled)) {
        bool was_position = position_running || dual_running;
        if (was_position) {
            fault_latched = true;
            last_fault_event = control_stalled ? EVENT_CONTROL_TIMEOUT :
                (loop_stalled ? EVENT_LOOP_TIMEOUT : EVENT_POSITION_TIMEOUT);
        }
        MotorTest_Stop();
        event = control_stalled ? EVENT_CONTROL_TIMEOUT : (loop_stalled ? EVENT_LOOP_TIMEOUT :
                (was_position ? EVENT_POSITION_TIMEOUT : EVENT_TIMED_STOP));
    } else if (armed && (uint32_t)(now - armed_at) >= ARM_TIMEOUT_MS) {
        armed = false;
        event = EVENT_ARM_EXPIRED;
    }
    /* Debug halt suspends CPU interrupt handling too: disconnect motor power before debugging. */
}

static void Reply(const char *text)
{
    ConsoleService_Reply(text,false);
}

typedef struct {
    bool running, armed, fault, moving;
    int pwm;
    MotorEvent fault_event;
} OutputSnapshot;

static OutputSnapshot ReadOutput(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    OutputSnapshot copy = {running, armed, fault_latched, position_running, commanded_percent, last_fault_event};
    if (!mask) __enable_irq();
    return copy;
}

static const char *StateName(OutputSnapshot output)
{
    if (output.fault) return "FAULT";
    if (output.moving) return "MOVING";
    if (output.running) return "JOG";
    if (output.armed) return "ARMED";
    if (position.status == POSITION_DONE) return "DONE";
    return PositionConfig_Valid(ActivePositionConfig()) ? "READY" : "UNCALIBRATED";
}

static void ReportStatus(void)
{
    OutputSnapshot output = ReadOutput();
    const char *reason = "NONE";
    if (output.fault) {
        if (output.fault_event == EVENT_CONTROL_TIMEOUT) reason = "CONTROL_TIMEOUT";
        else if (output.fault_event == EVENT_LOOP_TIMEOUT) reason = "LOOP_TIMEOUT";
        else if (output.fault_event == EVENT_POSITION_TIMEOUT) reason = "MOVE_TIMEOUT";
        else reason = PositionStatus_Name(position.status);
    }
    char text[320];
    snprintf(text, sizeof(text),
             "G431 MOTOR: running=%u armed=%u pwm=%d%% position=%s calibrated=%u fault=%u pos_um=%ld goal_um=%ld state=%s valid=%u age_ms=%lu fault_reason=%s driver=%s channel=%u\r\n",
             (unsigned)output.running, (unsigned)output.armed, output.pwm,
             output.moving ? "RUNNING" :
                (position.status == POSITION_RUNNING ? "CANCELLED" : PositionStatus_Name(position.status)),
             (unsigned)PositionConfig_Valid(ActivePositionConfig()), (unsigned)output.fault,
             (long)(position.measured_mm * 1000), (long)(position.goal_mm * 1000),
             StateName(output), (unsigned)measurement_valid,
             (unsigned long)(measurement_valid ? HAL_GetTick() - measurement_at : 0), reason,
             jog_driver ? "MDD10A" : "L298N", jog_driver ? jog_driver : 1);
    Reply(text);
}

static void ReportStats(void)
{
    char text[320];
    ConsoleServiceStats console=ConsoleService_ReadStats();
    snprintf(text, sizeof(text),
        "STATS period_ms=%u calls=%lu gap_max_ms=%lu late_max_ms=%lu skipped=%lu control_max_us=%lu task_max_us=%lu overruns=%lu uart_errors=%lu tx_peak=%u tx_drop=%lu clock_ok=%u\r\n",
        CONTROL_PERIOD_MS, (unsigned long)stats.calls, (unsigned long)stats.max_interval_ms,
        (unsigned long)stats.max_late_ms, (unsigned long)stats.missed_releases,
        (unsigned long)stats.max_control_us, (unsigned long)stats.max_task_us,
        (unsigned long)stats.overruns, (unsigned long)uart_error_count,
        console.peak, (unsigned long)console.dropped, (unsigned)RuntimePort_ClockReady());
    Reply(text);
}

static void ServiceTelemetry(uint32_t now)
{
    uint32_t late, skipped;
    if (!trace_enabled || !PeriodicJob_Take(&telemetry_job, now, &late, &skipped)) return;
    OutputSnapshot output = ReadOutput();
    char text[224];
    snprintf(text, sizeof(text),
        "TRACE t_ms=%lu raw=%u pos_um=%ld goal_um=%ld pwm=%d state=%s valid=%u age_ms=%lu\r\n",
        (unsigned long)now, last_raw, (long)(position.measured_mm * 1000),
        (long)(position.goal_mm * 1000), output.pwm, StateName(output),
        (unsigned)measurement_valid, (unsigned long)(measurement_valid ? now - measurement_at : 0));
    ConsoleService_Reply(text,true);
}

static void ReportSensor(void)
{
    /* Hand-operated inspection: SENSOR always cancels motor movement and ARM. */
    MotorTest_Stop();
    uint16_t raw;
    if (!SliderSensor_Read(&raw)) {
        measurement_valid = false;
        Reply("ERR SENSOR ADC unavailable or timed out\r\n");
        return;
    }

    char text[160];
    unsigned millivolts = ((unsigned)raw * 3300U + 2047U) / 4095U;
    float mm;
    last_raw = raw;
    measurement_at = HAL_GetTick();
    measurement_valid = SliderCalibration_Position(&position_config[0].calibration, raw, &mm);
    if (measurement_valid) {
        position.measured_mm = mm;
        snprintf(text,sizeof(text),"SENSOR raw=%u mv_est=%u pos_um=%ld (3V3 reference assumed)\r\n",
                 (unsigned)raw,millivolts,(long)(mm*1000));
    } else {
        snprintf(text,sizeof(text),"SENSOR raw=%u mv_est=%u pos=UNAVAILABLE (uncalibrated/outside points)\r\n",
                 (unsigned)raw,millivolts);
    }
    Reply(text);
}

static void ArmMotor(uint32_t now, uint32_t expected_epoch)
{
    if (fault_latched && !(dual_mode ?
        (SignalHealth_Ready(&dual_health[0],now) && SignalHealth_Ready(&dual_health[1],now)) :
        SignalHealth_Ready(&retry_health,now))) {
        Reply("ERR RETRY_WAIT fresh stable sensor required; retry ARM, no RESET\r\n");
        return;
    }
    if (running) {
        MotorTest_Stop();
        Reply("ERR RUNNING; stopped. ARM again\r\n");
        return;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (rx_abort || stop_epoch != expected_epoch) {
        if (!mask) __enable_irq();
        Reply("ERR cancelled; ARM again\r\n");
        return;
    }
    if (fault_latched) {
        fault_latched=false; last_fault_event=EVENT_NONE; event=EVENT_NONE;
        position.status=POSITION_IDLE;
    }
    armed_at = now;
    armed = true;
    if (!mask) __enable_irq();
    Reply("OK ARM (10s, one JOG)\r\n");
}

static void StartJog(const MotorCommand *command, uint32_t now)
{
    if (running) {
        MotorTest_Stop();
        Reply("ERR BUSY; stopped\r\n");
        return;
    }
    if (!armed || (uint32_t)(now - armed_at) >= ARM_TIMEOUT_MS) {
        armed = false;
        Reply("ERR send ARM first\r\n");
        return;
    }
    if ((uint32_t)(now - stopped_at) < RESTART_DELAY_MS) {
        Reply("ERR wait 100ms after stop\r\n");
        return;
    }

    unsigned duty = (unsigned)(command->percent > 0 ?
                               command->percent : -command->percent);
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    if (!armed || fault_latched || rx_abort || (uint32_t)(HAL_GetTick()-armed_at)>=ARM_TIMEOUT_MS) {
        if (!interrupt_mask) __enable_irq();
        Reply("ERR cancelled; ARM again\r\n");
        return;
    }
    armed = false;             /* One ARM allows exactly one JOG. */
    commanded_percent = command->percent;
    applied_direction = command->percent > 0 ? 1 : -1;
    deadline = now + command->duration_ms;
    if (jog_driver) {
        TIM3->CCR1 = 0;
        TIM3->CCER &= ~TIM_CCER_CC1E;
        GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16;
        LiveIO_Jog(jog_driver - 1U, command->percent);
    } else {
        LiveIO_Off();
        GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16;
        GPIOB->BSRR = command->percent > 0 ? GPIO_PIN_0 : GPIO_PIN_1;
        TIM3->CCR1 = duty * PWM_PERIOD_COUNTS / 100U;
        TIM3->CCER |= TIM_CCER_CC1E;
    }
    running = true;

    if (!interrupt_mask) {
        __enable_irq();
    }
    Reply("OK JOG\r\n");
}


static void ReportCalibration(void)
{
    const PositionConfig *c = ActivePositionConfig();
    char text[180];
    snprintf(text,sizeof(text),
        "CAL A ready=%u adc=%u,%u,%u mm_x1000=%ld,%ld,%ld sign=%d (0=unverified); B=reserved\r\n",
        (unsigned)PositionConfig_Valid(c), c->calibration.adc[0],c->calibration.adc[1],c->calibration.adc[2],
        (long)(c->calibration.mm[0]*1000),(long)(c->calibration.mm[1]*1000),
        (long)(c->calibration.mm[2]*1000),c->motor_sign);
    Reply(text);
}

static void StartMove(float target, uint32_t now)
{
    if ((jog_driver==1 && !bench1) || (jog_driver==2 && !bench2) ||
        (!jog_driver && bench1)) {
        MotorTest_Stop();
        Reply("ERR position routing: select matching DRIVER MDD10A 1/2 then BENCH1/2 ON\r\n");
        return;
    }
    if (running) { MotorTest_Stop(); Reply("ERR BUSY; stopped\r\n"); return; }
    if (fault_latched) { Reply("ERR RETRY_WAIT send ARM after sensor recovery\r\n"); return; }
    if (!armed || (uint32_t)(now-armed_at) >= ARM_TIMEOUT_MS) {
        armed=false;Reply("ERR send ARM first\r\n");return;
    }
    if (!PositionConfig_Valid(ActivePositionConfig())) {
        MotorTest_Stop();Reply("ERR CALIBRATION required: Core/Inc/position_config.h\r\n");return;
    }
    if ((uint32_t)(now-stopped_at)<RESTART_DELAY_MS) {Reply("ERR wait 100ms after stop\r\n");return;}
    uint16_t raw;
    measurement_valid = false;
    if (!ReadPositionSensor(&raw)) {MotorTest_Stop();Reply("ERR SENSOR unavailable\r\n");return;}
    now = HAL_GetTick();
    PositionStatus result = Position_Start(&position,ActivePositionConfig(),target,raw,now);
    if (result != POSITION_RUNNING) {
        MotorTest_Stop();
        char text[96];snprintf(text,sizeof(text),"ERR MOVE %s\r\n",PositionStatus_Name(result));Reply(text);return;
    }
    uint32_t mask=__get_PRIMASK();__disable_irq();
    bool allowed=armed && !fault_latched && !rx_abort && (uint32_t)(now-armed_at)<ARM_TIMEOUT_MS;
    if (allowed) {
        armed = false;
        previous_control_start = control_release_ms;
        last_raw = raw;
        measurement_at = now;
        measurement_valid = true;
        deadline = now + ActivePositionConfig()->max_move_ms;
        last_task = now;
        last_control_completed = now;
        position_running = true;
        running = true;
    }
    if (!mask) __enable_irq();
    if (!allowed) {position.status=POSITION_IDLE;Reply("ERR cancelled; ARM again\r\n");return;}
    Reply(bench1 ? "OK MOVE MDD10A1 SENSOR1\r\n" :
          (bench2 ? (jog_driver==2 ? "OK MOVE MDD10A2 SENSOR2\r\n" : "OK MOVE SENSOR2\r\n") : "OK MOVE A\r\n"));
}

/* Called with interrupts masked and only while position_running remains true. */
static void ApplyPositionPwm(int percent, uint32_t now)
{
    int direction = percent > 0 ? 1 : (percent < 0 ? -1 : 0);
    if (direction != applied_direction) {
        if (applied_direction != 0) {
            LiveIO_Off();
            TIM3->CCR1=0;TIM3->CCER &= ~TIM_CCER_CC1E;
            GPIOB->BSRR=(GPIO_PIN_0|GPIO_PIN_1)<<16;
            applied_direction=0;commanded_percent=0;stopped_at=now;
            return; /* 100ms off before any restart/reversal */
        }
        if (direction && (uint32_t)(now-stopped_at)<RESTART_DELAY_MS) return;
        GPIOB->BSRR=(GPIO_PIN_0|GPIO_PIN_1)<<16;
        if (direction && !jog_driver) GPIOB->BSRR=direction>0?GPIO_PIN_0:GPIO_PIN_1;
        applied_direction=direction;
    }
    commanded_percent=percent;
    if (jog_driver) {
        /* Register writes only: no HAL_Delay, UART wait or extra task. */
        TIM3->CCR1=0; TIM3->CCER &= ~TIM_CCER_CC1E;
        LiveIO_SingleWrite(jog_driver-1U,percent);
        return;
    }
    unsigned duty=(unsigned)(percent<0?-percent:percent);
    TIM3->CCR1=duty*PWM_PERIOD_COUNTS/100U;
    if (duty) TIM3->CCER |= TIM_CCER_CC1E;
    else TIM3->CCER &= ~TIM_CCER_CC1E;
}

static void ServicePosition(uint32_t now)
{
    if (!position_running) return;
    if ((int32_t)(now-position.last_ms)<=0) return; /* No new sample interval yet. */
    uint32_t interval=now-previous_control_start;
    if (interval==0) return; /* MOVE-start release: first update is the next task release. */
    uint32_t late=interval>CONTROL_PERIOD_MS ? interval-CONTROL_PERIOD_MS : 0;
    uint32_t skipped=interval/CONTROL_PERIOD_MS;
    skipped=skipped>0 ? skipped-1 : 0;

    uint32_t started_cycles = RuntimePort_Cycles();
    uint32_t started_ms = HAL_GetTick();
    ++stats.calls;
    RecordMax(&stats.max_interval_ms, now - previous_control_start);
    RecordMax(&stats.max_late_ms, late);
    stats.missed_releases += skipped;
    previous_control_start = now;

    /* Capture the last accepted observation and applied command before a
     * fault clears PWM. Diagnostics are queued only AFTER the motor is off. */
    uint16_t previous_raw = last_raw;
    uint32_t sample_dt_ms = 0;
    int pwm_before = commanded_percent;
    float step_mm = 0, jump_limit_mm = 0;
    PositionStatus result;
    if (skipped != 0) {
        /* A complete control period was missed. Do not apply stale output. */
        result = POSITION_TIMING_ERROR;
    } else {
        uint16_t raw = 0;
        bool ok = ReadPositionSensor(&raw);
        now = HAL_GetTick();
        sample_dt_ms = now - position.last_ms;
        float previous_mm = position.previous_mm;
        result = Position_Update(&position, ActivePositionConfig(), ok, raw, now);
        step_mm = fabsf(position.measured_mm - previous_mm);
        jump_limit_mm = 0.25f + ActivePositionConfig()->max_sensor_speed_mm_s *
                        (sample_dt_ms * 0.001f);
        float observed_mm;
        measurement_valid = ok && SliderCalibration_Position(
            &ActivePositionConfig()->calibration, raw, &observed_mm);
        /* position.measured_mm belongs to the filtered controller state. */
        if (ok) { last_raw = raw; measurement_at = now; }
        uint32_t elapsed_us = RuntimePort_ElapsedUs(started_cycles);
        RecordMax(&stats.max_control_us, elapsed_us);
        if ((uint32_t)(now - started_ms) + late >= CONTROL_PERIOD_MS ||
            elapsed_us >= CONTROL_PERIOD_MS * 1000U) {
            result = POSITION_TIMING_ERROR;
        }
    }
    if (result == POSITION_TIMING_ERROR) {
        ++stats.overruns;
        position.status = result;
        position.pwm_percent = 0;
    }

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool still_running = position_running;
    if (still_running) {
        if (result == POSITION_RUNNING) {
            ApplyPositionPwm(position.pwm_percent, now);
            last_control_completed = now;
        } else {
            if (result != POSITION_DONE) fault_latched = true;
            MotorTest_Stop();
        }
    }
    if (!mask) __enable_irq();
    RecordMax(&stats.max_control_us, RuntimePort_ElapsedUs(started_cycles));
    if (still_running && result != POSITION_RUNNING) {
        char text[224];
        if (result == POSITION_JUMP_ERROR) {
            snprintf(text, sizeof(text),
                "ERR MOVE SENSOR_JUMP pos_um=%ld raw_prev=%u raw_now=%u adc_used=%u dt_ms=%lu pwm_before=%d step_um=%ld limit_um=%ld\r\n",
                (long)(position.measured_mm * 1000), previous_raw, last_raw, position.feedback_adc,
                (unsigned long)sample_dt_ms, pwm_before,
                (long)lroundf(step_mm * 1000), (long)lroundf(jump_limit_mm * 1000));
        } else {
            snprintf(text, sizeof(text), "%s MOVE %s pos_um=%ld\r\n",
                     result == POSITION_DONE ? "DONE" : "ERR", PositionStatus_Name(result),
                     (long)(position.measured_mm * 1000));
        }
        Reply(text);
    }
}

/* Both controllers run once per existing 10ms ControlTask release. They share
 * a command, not feedback: an arrived motor stays off while the other finishes.
 * No synchronous motor waits and no independent output writer/task are added. */
static void StartDualPosition(unsigned target,uint32_t now,uint32_t epoch)
{
    if (running) {MotorTest_Stop();Reply("ERR BUSY; stopped\r\n");return;}
    if (fault_latched) {Reply("ERR RETRY_WAIT send ARM after sensor recovery\r\n");return;}
    if (!armed || now-armed_at>=ARM_TIMEOUT_MS) {Reply("ERR send ARM first\r\n");return;}
    if (now-stopped_at<RESTART_DELAY_MS) {Reply("ERR wait 100ms after stop\r\n");return;}
    uint16_t raw[2]={0};
    bool ok0=SliderSensor_ReadAxis(0,&raw[0]);
    bool ok1=SliderSensor_ReadAxis(1,&raw[1]);
    if (!ok0 || !ok1) {MotorTest_Stop();Reply("ERR DUAL SENSOR unavailable\r\n");return;}
    now=HAL_GetTick();
    PositionStatus r=DualPosition_Start(&dual,target,raw,now);
    if (r!=POSITION_RUNNING) {MotorTest_Stop();Reply("ERR DUAL configuration/target/sensor\r\n");return;}
    uint32_t mask=__get_PRIMASK();__disable_irq();
    bool allowed=armed && !fault_latched && !rx_abort && epoch==stop_epoch && now-armed_at<ARM_TIMEOUT_MS;
    if (allowed) {
        armed=false;dual_running=true;running=true;
        dual_release=control_release_ms;
        last_control_completed=last_task=now;
        deadline=now+60000U;
    }
    if (!mask) __enable_irq();
    Reply(allowed?"OK DUAL MOVE\r\n":"ERR cancelled; ARM again\r\n");
}

static void ServiceDualPosition(uint32_t release)
{
    if (!dual_running || release==dual_release) return;
    uint32_t epoch=stop_epoch,interval=release-dual_release;
    dual_release=release;
    uint32_t cycles=RuntimePort_Cycles();
    uint16_t raw[2]={0};
    bool ok[2];
    ok[0]=SliderSensor_ReadAxis(0,&raw[0]);
    ok[1]=SliderSensor_ReadAxis(1,&raw[1]);
    uint32_t now=HAL_GetTick();
    PositionStatus r;
    if (interval>=2U*CONTROL_PERIOD_MS || now-release>=CONTROL_PERIOD_MS)
        r=POSITION_TIMING_ERROR;
    else r=DualPosition_Update(&dual,ok,raw,now);
    ++stats.calls;
    RecordMax(&stats.max_interval_ms,interval);
    RecordMax(&stats.max_late_ms,interval>CONTROL_PERIOD_MS?interval-CONTROL_PERIOD_MS:0);
    if (r==POSITION_TIMING_ERROR) ++stats.overruns;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    bool active=dual_running && epoch==stop_epoch;
    if (active) {
        if (r==POSITION_RUNNING) {
            LiveIO_Write(dual.axis[0].pwm_percent*.01f,dual.axis[1].pwm_percent*.01f,now);
            last_control_completed=now;
        } else {
            if (r!=POSITION_DONE) fault_latched=true;
            MotorTest_Stop();
        }
    }
    if (!mask) __enable_irq();
    RecordMax(&stats.max_control_us,RuntimePort_ElapsedUs(cycles));
    if (active && r!=POSITION_RUNNING) {
        char text[160];
        snprintf(text,sizeof text,"%s DUAL %s fault_axis=%u adc1=%u adc2=%u\r\n",
            r==POSITION_DONE?"DONE":"ERR",PositionStatus_Name(r),dual.fault_axis,
            dual.axis[0].feedback_adc,dual.axis[1].feedback_adc);
        Reply(text);
    }
}

static InputTrial trial;
static uint32_t trial_epoch;

/* Observe and run use the same physical inputs. SIM commands never enter here. */
/* Fresh stopped samples are collected without blocking ControlTask or UART. */
static void ServiceRetryHealth(uint32_t now)
{
    if (!fault_latched || running || live_mode) { memset(&retry_health,0,sizeof retry_health); return; }
    if (dual_mode) {
        for (unsigned i=0;i<2;i++) {
            uint16_t raw=0;
            bool ok=SliderSensor_ReadAxis(i,&raw) && raw<=4095;
            SignalHealth_Update(&dual_health[i],ok,raw,HAL_GetTick());
        }
        return;
    }
    if (retry_health.sampled && now-retry_health.last_ms<10) return;
    uint16_t raw=0;float mm;
    bool ok=ReadPositionSensor(&raw) && SliderCalibration_Position(&ActivePositionConfig()->calibration,raw,&mm);
    SignalHealth_Update(&retry_health,ok,raw,HAL_GetTick());
}

static void ServiceLive(void)
{
    if (!live_mode) return;
    uint32_t now=HAL_GetTick();
    if (live_epoch!=stop_epoch) {live_mode=false; return;}
    ReceiverCommand command;
    AppImuSnapshot imu;
    bool have_command;
    if (team_mode) {
        TeamLinkSnapshot snapshot=TeamLink_Read();
        command=snapshot.radio; have_command=snapshot.have_radio;
        imu=snapshot.imu;
    } else {
        have_command=TrialReceiver_ReadCommand(&command);
        imu=AppInputs_ReadImu();
    }
    if (have_command) {
        live_input.radio_ok=command.command_valid;
        live_input.radio_ms=command.received_ms;
        if (command.command_valid) {
            live_input.requested=command.requested;
            live_input.stick=command.stick;
        }
    }
    live_input.imu_ok=imu.valid;
    live_input.pitch_deg=imu.pitch_deg;
    live_input.imu_ms=imu.sampled_ms;
    now=HAL_GetTick();
    /* The RTOS release sets the period; do not add a second drifting gate. */
    live_input.adc_ok=SliderSensor_ReadAxis(0,&live_input.adc[0]) &&
                      SliderSensor_ReadAxis(1,&live_input.adc[1]);
    now=HAL_GetTick();
    uint32_t dt=now-live_tick;
    live_tick=now;
    if (!live_permit) return;
    if (live_timeout) {live_timeout=false;Sy_Fail(&live_control,SY_TIMING);}
    Sy_Tick(&live_control,&live_cfg,&live_input,now,dt*.001f);
    if (!live_control.armed && now-live_request>1500 && live_control.mode!=SY_FAULT)
        Sy_Fail(&live_control,SY_RADIO);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (live_control.mode==SY_FAULT) {LiveIO_Off();live_heartbeat=now;}
    else if (live_epoch==stop_epoch && live_permit && !rx_abort) {
        LiveIO_Write(live_control.pwm[0],live_control.pwm[1],now);
        live_heartbeat=now;
    }
    if (!mask) __enable_irq();
}

static void LiveStatus(void)
{
    static const char *mode[]={"OFF","UP","HOVER","DOWN","FAULT"};
    static const char *reason[]={"OK","CONFIG","RADIO","SENSOR","IMU","LIMIT","STALL","TIMING","TIMEOUT"};
    uint32_t now=HAL_GetTick(); char s[240];
    snprintf(s,sizeof s,"LIVE permit=%u armed=%u fault=%s radio=%u sw=%s ch4_pct=%d adc_ok=%u raw=%u,%u imu=%u pitch_mdeg=%ld target_um=%ld,%ld pwm_req_pct=%d,%d\r\n",
        live_permit,live_control.armed,reason[live_control.fault],live_input.radio_ok&&now-live_input.radio_ms<=300,
        mode[live_input.requested],(int)(100*live_input.stick),live_input.adc_ok,live_input.adc[0],live_input.adc[1],
        live_input.imu_ok&&now-live_input.imu_ms<=100,live_input.imu_ok?lroundf(live_input.pitch_deg*1000):0L,
        lroundf(live_control.target[0]*1000),lroundf(live_control.target[1]*1000),
        (int)(100*live_control.pwm[0]),(int)(100*live_control.pwm[1]));
    Reply(s);
}

static void ServiceTrial(uint32_t now)
{
    if (trial.source==TRIAL_OFF) return;
    if (trial_epoch!=stop_epoch) {
        Trial_Start(&trial,TRIAL_OFF,0,now);
        return;
    }
    ReceiverCommand command;
    if (trial.source==TRIAL_RC && TrialReceiver_ReadCommand(&command))
        Trial_ReceiverCommand(&trial,&command);
    Trial_Tick(&trial,now);
}

/* Inspection command: stop/disarm before sequential ADC reads. No LIVE pin setup. */
static void ReportSensorBoth(void)
{
    MotorTest_Stop();
    uint16_t raw[2];
    for (unsigned axis=0; axis<2; ++axis) {
        if (!SliderSensor_ReadAxis(axis,&raw[axis])) {
            char error[80];
            measurement_valid=false;
            snprintf(error,sizeof error,"ERR SENSORS axis=%u ADC unavailable or timed out\r\n",axis+1);
            Reply(error);
            return;
        }
    }
    char text[80];
    snprintf(text,sizeof text,"SENSORS raw1=%u raw2=%u\r\n",(unsigned)raw[0],(unsigned)raw[1]);
    Reply(text);
}

static void HandleCommand(uint32_t expected_epoch)
{
    if (expected_epoch != stop_epoch) return;
    MotorCommand command = MotorCommand_Parse(command_line);
    uint32_t now = HAL_GetTick();

    if (!strncmp(command_line,"DRIVER ",7)) {
        if (live_mode || trial.source != TRIAL_OFF) {
            MotorTest_Stop();
            Reply("ERR send STOP then DRIVER; previous mode stopped\r\n");
            return;
        }
        MotorTest_Stop();
        bench1=bench2=false; dual_mode=false;
        memset(&retry_health,0,sizeof retry_health);
        if (!strcmp(command_line,"DRIVER L298N")) {
            jog_driver=0;
            Reply("OK DRIVER L298N\r\n");
        } else if (!strcmp(command_line,"DRIVER MDD10A 1") ||
                   !strcmp(command_line,"DRIVER MDD10A 2")) {
            LiveIO_InitOutputs();
            jog_driver=(unsigned)(command_line[14]-'0');
            Reply(jog_driver==1 ? "OK DRIVER MDD10A 1 JOG_ONLY\r\n" :
                                  "OK DRIVER MDD10A 2 JOG_ONLY\r\n");
        } else {
            Reply("ERR DRIVER L298N | DRIVER MDD10A 1 | DRIVER MDD10A 2\r\n");
        }
        return;
    }
    if (!strcmp(command_line,"SENSOR BOTH")) { ReportSensorBoth(); return; }

    char encoder_reply[240];
    if (EncoderService_Command(command_line,running || position_running || live_permit || armed,now,
                               encoder_reply,sizeof encoder_reply)) {
        Reply(encoder_reply); return;
    }
    if (live_mode && !strcmp(command_line,"PITCH STATUS")) {
        char text[300];
        snprintf(text,sizeof text,"PITCH control=ANGLE_PWM center_ready=%u limited=%u hold=%u rate_ok=%u target_mdeg=%ld actual_mdeg=%ld rate_mdeg_s=%ld pwm_req_pct=%d,%d permit=%u\r\n",
            live_control.hover_position_ready,live_control.pitch_limited,
            live_control.pitch_holding,live_control.pitch_rate_ready,
            lroundf(live_control.pitch_target*1000),lroundf(live_input.pitch_deg*1000),
            lroundf(live_control.pitch_rate*1000),(int)(100*live_control.pwm[0]),
            (int)(100*live_control.pwm[1]),live_permit);
        Reply(text);return;
    }
    if (!strcmp(command_line,"RTOS")) {
        char text[640]; AppTasks_FormatStats(text,sizeof text); Reply(text); return;
    }
    if (!strcmp(command_line,"HELP ALL")) {
        Reply("MDD position: DRIVER MDD10A 1 | BENCH1 ON/OFF/STATUS | ARM then BUOY UP/HOVER/DOWN (PA0)\r\n");
        Reply("Fixture: DRIVER MDD10A 2 or L298N | BENCH2 ON/OFF/STATUS | ARM then BUOY UP/HOVER/DOWN (PA1)\r\n");
        Reply("Inspection: SENSOR BOTH (stop/disarm; PA0 and PA1 raw ADC)\r\n");
        Reply("Angle: PITCH STATUS (SIM/RC/LIVE); HOVER uses angle-driven PWM\r\n");
        Reply("Encoder: ENC HELP (observation only, no motor authority)\r\n");
        Reply("Runtime: RTOS (task timing and minimum stack headroom)\r\n");
        Reply("Legacy: STOP | ARM | JOG signed_percent milliseconds | STATUS | SENSOR\r\n");
        Reply("Position: CAL | MOVE mm | STATS | TRACE ON/OFF (calibration required)\r\n");
        Reply("Preview: MODE SIM | MODE RC [switch channel] | CHANNELS | SW UP/HOVER/DOWN | STICK percent | PITCH deg | ADC front rear | LINK OFF\r\n");
        Reply("Physical: MODE LIVE n | MODE TEAM | STATUS | LIVE ARM (measured config required); STOP returns to legacy mode\r\n");
        Reply("TEAM: external SwC CH6=1000 UP/1500 HOVER/2000 DOWN; LH CH4 right=nose up; TEAM STATUS\r\n");
        return;
    }
    if (!strcmp(command_line,"TEAM STATUS")) {
        TeamLinkSnapshot s=TeamLink_Read(); char text[240];
        snprintf(text,sizeof text,"TEAM active=%u frame=%u valid=%u CH4=%u CH6=%u radio_age_ms=%lu imu=%u imu_age_ms=%lu pitch_mdeg=%ld\r\n",
            live_mode && team_mode && live_epoch==stop_epoch,s.have_radio,s.radio.command_valid,
            s.radio.channels[3],s.radio.channels[5],(unsigned long)(now-s.radio.received_ms),
            s.imu.valid,(unsigned long)(now-s.imu.sampled_ms),
            s.imu.valid?lroundf(s.imu.pitch_deg*1000):0L);
        Reply(text);return;
    }
    char trial_reply[200];
    if (!strncmp(command_line,"MODE ",5)) {
        dual_mode=false;
        MotorTest_Stop();
        live_mode=false; team_mode=false;
        AppInputs_RequestImu(false);
        trial_epoch=stop_epoch;
        Trial_Start(&trial,TRIAL_OFF,0,now);
        TrialReceiver_Clear();
        if (!strcmp(command_line,"MODE TEAM")) {
            TeamLink_Clear(); /* require reports received AFTER selecting mode */
            LiveIO_InitOutputs();
            live_epoch=trial_epoch;
            Sy_Init(&live_control);
            memset(&live_input,0,sizeof live_input);
            live_tick=HAL_GetTick(); team_mode=true; live_mode=true;
            Reply("OK TEAM observe only; external CH4/CH6 + pitch; TEAM STATUS; LIVE ARM requires measured configuration\r\n");
            return;
        }
        if (!strncmp(command_line,"MODE LIVE ",10)) {
            const char *n=command_line+10;
            if (*n<'1'||*n>'9'||strlen(n)>2||(n[1]&&(n[1]<'0'||n[1]>'9'))) {
                Reply("ERR MODE LIVE 1..14 (not CH4)\r\n");return;
            }
            unsigned channel=n[1]?10U*(n[0]-'0')+n[1]-'0':(unsigned)(n[0]-'0');
            if(channel>14||channel==4||!TrialReceiver_Init()){Reply("ERR channel/receiver\r\n");return;}
            LiveIO_InitOutputs(); /* New driver pins are only claimed by explicit LIVE mode. */
            live_epoch=trial_epoch;
            TrialReceiver_Configure(channel,LIVE_REVERSE_SWITCH,LIVE_REVERSE_CH4);
            Sy_Init(&live_control);
            memset(&live_input,0,sizeof live_input);
            AppInputs_RequestImu(true); /* ImuTask initializes asynchronously. */
            live_tick=HAL_GetTick(); live_mode=true;
            Reply("OK LIVE observe only; STATUS; LIVE ARM requires measured configuration\r\n");
            return;
        }
        if (!Trial_Command(&trial,command_line,now,trial_reply,sizeof trial_reply)) {
            Reply("ERR MODE SIM | MODE RC [switch channel]\r\n");
            return;
        }
        if (trial.source==TRIAL_RC && !TrialReceiver_Init()) {
            Trial_Start(&trial,TRIAL_OFF,0,now);
            Reply("ERR USART1 init\r\n");
            return;
        }
        if(trial.source==TRIAL_RC) TrialReceiver_Configure(trial.switch_channel,false,false);
        Reply("OK preview; physical motors OFF\r\n");
        return;
    }
    if (live_mode && command.type!=CMD_STOP && command.type!=CMD_HELP) {
        if (command.type==CMD_STATUS) {LiveStatus();return;}
        if (strcmp(command_line,"LIVE ARM")) {Reply("ERR live: STATUS | LIVE ARM | STOP\r\n");return;}
        if (live_permit || live_control.fault!=SY_OK || fault_latched || !live_cfg.enable || !Sy_Validate(&live_cfg)) {
            Reply("ERR already enabled, fault or unconfirmed live_config.h\r\n");return;
        }
        if(!live_input.radio_ok || now-live_input.radio_ms>300 || live_input.requested!=SY_HOVER ||
           fabsf(live_input.stick)>.05f || !live_input.adc_ok ||
           (live_cfg.angle_enable && (!live_input.imu_ok || now-live_input.imu_ms>100))) {
            Reply("ERR check receiver neutral, both sliders and fresh IMU\r\n");return;
        }
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        bool ok=live_epoch==stop_epoch&&!rx_abort;
        if(ok){live_timeout=false;live_heartbeat=live_request=now;live_permit=true;}
        if(!mask)__enable_irq();
        Reply(ok?"OK LIVE ARM; hold neutral 500ms\r\n":"ERR stopped\r\n");
        return;
    }
    if (trial.source!=TRIAL_OFF && command.type!=CMD_STOP && command.type!=CMD_HELP) {
        Trial_Command(&trial,command_line,now,trial_reply,sizeof trial_reply);
        Reply(trial_reply);
        return;
    }
    if (!strcmp(command_line,"DUAL ON")) {
        MotorTest_Stop();
        bench1=bench2=false; dual_mode=true;
        memset(dual_health,0,sizeof dual_health);
        DualPosition_Init(&dual);
        LiveIO_InitOutputs();
        Reply("OK DUAL ON MDD10A1/PA0 MDD10A2/PA1 sign=-1,-1 PWM=50; ARM per move\r\n");
        return;
    }
    if (!strcmp(command_line,"DUAL OFF")) {
        MotorTest_Stop(); dual_mode=false;
        Reply("OK DUAL OFF\r\n");return;
    }
    if (!strcmp(command_line,"DUAL STATUS")) {
        char text[240];
        snprintf(text,sizeof text,"DUAL enabled=%u running=%u fault=%u axis1=%s adc1=%u pwm1=%d axis2=%s adc2=%u pwm2=%d\r\n",
            dual_mode,dual_running,fault_latched,
            PositionStatus_Name(dual.axis[0].status),dual.axis[0].feedback_adc,
            dual_running?dual.axis[0].pwm_percent:0,
            PositionStatus_Name(dual.axis[1].status),dual.axis[1].feedback_adc,
            dual_running?dual.axis[1].pwm_percent:0);
        Reply(text);return;
    }
    if (!strncmp(command_line,"DUAL ",5)) {
        unsigned target=3;
        if (!strcmp(command_line+5,"UP")) target=0;
        if (!strcmp(command_line+5,"HOVER")) target=1;
        if (!strcmp(command_line+5,"DOWN")) target=2;
        if (target>2 || !dual_mode) {
            MotorTest_Stop();Reply("ERR DUAL ON then ARM then DUAL UP/HOVER/DOWN\r\n");return;
        }
        StartDualPosition(target,now,expected_epoch);return;
    }
    if (dual_mode && (!strncmp(command_line,"BUOY ",5) ||
        !strncmp(command_line,"JOG ",4) || !strncmp(command_line,"MOVE ",5))) {
        MotorTest_Stop();Reply("ERR DUAL active; use DUAL UP/HOVER/DOWN or select DRIVER\r\n");return;
    }

    if (!strcmp(command_line,"BENCH1 ON") || !strcmp(command_line,"BENCH1 OFF") ||
        !strcmp(command_line,"BENCH2 ON") || !strcmp(command_line,"BENCH2 OFF")) {
        MotorTest_Stop();
        dual_mode=false;
        bool first=command_line[5]=='1';
        bool enable=!strcmp(command_line+7,"ON");
        bench1=bench2=false;
        if (enable && ((first && jog_driver!=1) || (!first && jog_driver!=0 && jog_driver!=2))) {
            Reply("ERR BENCH1 requires MDD10A 1; BENCH2 requires MDD10A 2 or L298N\r\n");return;
        }
        bench1=first && enable; bench2=!first && enable;
        bench_config = position_config[first?0:1];
        bench_config.motor_sign = -1; /* Negative JOG fills; each fixture assumes ADC increases on filling. */
        bench_config.safe_min_mm = 0;
        bench_config.allow_adc_endpoints = true;
        bench_config.adc_tolerance = 5;
        bench_config.median_adc = true;
        bench_config.safe_max_mm = 58.48f;
        bench_config.kp = 15; bench_config.ki = 5;
        bench_config.max_pwm_percent = 50;
        bench_config.target_rate_mm_s = 3;
        bench_config.stall_ms = DEMO_STALL_MS;
        bench_config.noise_tolerant = true;
        memset(&retry_health,0,sizeof retry_health);
        /* Continuous arrival profile shared by explicit one-motor fixtures.
         * JOG settings and dual-axis LIVE gains remain separate. */
        bench_config.pulse_drive = false;
        bench_config.continuous_drive = true;
        if (first) {
            Reply(bench1 ? "OK BENCH1 MDD10A1 PA0 sign=-1 drive=CONTINUOUS50 UP=0 HOVER=522 DOWN=4095; ARM per move\r\n" : "OK BENCH1 OFF\r\n");
            return;
        }
        if (bench2 && jog_driver==2) {
            Reply("OK BENCH2 MDD10A2 PA1 sign=-1 drive=CONTINUOUS50 UP=0 HOVER=529 DOWN=4095; ARM per move\r\n");
            return;
        }
        Reply(bench2 ? "OK BENCH2 PA1 sign=-1 drive=CONTINUOUS50 stop=ARRIVAL_OR_CROSSING noise=20 retry=ARM; ADC=0..4095 center+/-5; ARM per move\r\n" : "OK BENCH2 OFF\r\n");
        return;
    }
    if (!strcmp(command_line,"BENCH1 STATUS")) {
        char text[180];
        snprintf(text,sizeof text,"BENCH1 enabled=%u driver=MDD10A channel=1 sensor=1 UP_adc=0 HOVER_adc=522 DOWN_adc=4095; fixture targets, not neutral buoyancy\r\n",bench1);
        Reply(text);return;
    }
    if (!strcmp(command_line,"BENCH2 STATUS")) {
        char text[210];
        snprintf(text,sizeof text,"BENCH2 enabled=%u driver=%s channel=%u sensor=2 UP_adc=0 HOVER_adc=529 DOWN_adc=4095; fixture targets, not full stroke/neutral buoyancy\r\n",bench2,jog_driver==2?"MDD10A":"L298N",jog_driver==2?2U:1U);
        Reply(text);return;
    }
    if (!strncmp(command_line,"BUOY ",5)) {
        uint16_t target_raw;
        if (!bench1 && !bench2) { MotorTest_Stop(); Reply("ERR BENCH1 ON (MDD1/PA0) or BENCH2 ON (MDD2 or L298N/PA1) required\r\n");return; }
        if (!strcmp(command_line,"BUOY UP")) target_raw=0;
        else if (!strcmp(command_line,"BUOY HOVER")) target_raw=bench_config.calibration.adc[1];
        else if (!strcmp(command_line,"BUOY DOWN")) target_raw=4095;
        else { MotorTest_Stop();Reply("ERR BUOY UP|HOVER|DOWN\r\n");return; }
        float target_mm;
        if (!SliderCalibration_Position(&bench_config.calibration,target_raw,&target_mm)) {
            MotorTest_Stop();Reply("ERR BENCH2 calibration\r\n");return;
        }
        StartMove(target_mm,now);
        return;
    }
    switch (command.type) {
    case CMD_STOP:
        MotorTest_Stop();
        Reply("OK STOP\r\n");
        break;
    case CMD_STATUS:
        ReportStatus();
        break;
    case CMD_SENSOR:
        ReportSensor();
        break;
    case CMD_ARM:
        ArmMotor(now, expected_epoch);
        break;
    case CMD_JOG:
        StartJog(&command, now);
        break;
    case CMD_MOVE:
        StartMove(command.target_mm,now);
        break;
    case CMD_CAL:
        ReportCalibration();
        break;
    case CMD_STATS:
        ReportStats();
        break;
    case CMD_TRACE_ON:
        trace_enabled = true;
        PeriodicJob_Start(&telemetry_job, now, TELEMETRY_PERIOD_MS);
        Reply("OK TRACE ON (200ms, last measurement; no movement)\r\n");
        break;
    case CMD_TRACE_OFF:
        trace_enabled = false;
        Reply("OK TRACE OFF\r\n");
        break;
    case CMD_HELP: {
        char limits[96];
        Reply("STOP | ARM | JOG signed_percent milliseconds | STATUS | SENSOR\r\n");
        Reply("DRIVER L298N | DRIVER MDD10A 1 | DRIVER MDD10A 2 (timed JOG only)\r\n");
        snprintf(limits, sizeof(limits),
                 "JOG: +/-1..100%%, %u..%ums; ARM once per JOG; ! stops immediately\r\n",
                 MOTOR_JOG_MIN_MS, MOTOR_JOG_MAX_MS);
        Reply(limits);
        break;
    }
    default:
        MotorTest_Stop();
        Reply("ERR command; stopped. HELP for syntax\r\n");
        break;
    }
}

static void ReportPendingEvent(void)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    MotorEvent notice = event;
    event = EVENT_NONE;
    if (!interrupt_mask) {
        __enable_irq();
    }

    if (notice == EVENT_TIMED_STOP) {
        Reply("DONE STOP\r\n");
    } else if (notice == EVENT_LOOP_TIMEOUT) {
        Reply("ERR LOOP TIMEOUT; stopped\r\n");
    } else if (notice == EVENT_ARM_EXPIRED) {
        Reply("ARM EXPIRED\r\n");
    } else if (notice == EVENT_POSITION_TIMEOUT) {
        Reply("ERR MOVE_TIMEOUT; stopped; inspect then reset\r\n");
    } else if (notice == EVENT_CONTROL_TIMEOUT) {
        Reply("ERR CONTROL_TIMEOUT; stopped; inspect then reset\r\n");
    }
}

static void ServiceRx(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool aborted=rx_abort, error=rx_error;
    rx_abort=false; rx_error=false;
    if (!mask) __enable_irq();
    if(aborted) Reply(error ? "ERR UART/input; stopped; send newline then ARM\r\n" : "OK STOP\r\n");
    /* At most one complete command per release. Epoch checks invalidate queued
     * ARM/JOG after any stop; STOP itself bypasses this queue. */
    AppCommandMessage message;
    for(unsigned i=0;i<APP_COMMAND_DEPTH;i++) {
        if(!AppTransport_Receive(&message)) break;
        if(message.epoch!=stop_epoch) continue;
        memcpy(command_line,message.line,sizeof command_line);
        command_line[sizeof command_line-1]=0;
        /* The handler rechecks ARM/epoch at hardware writes. */
        HandleCommand(message.epoch);
        break;
    }
}

void ControlApp_Step(void)
{
    uint32_t started = RuntimePort_Cycles();
    control_release_ms = HAL_GetTick();
    last_task = control_release_ms;
    if (live_mode && live_epoch != stop_epoch) {
        live_mode=false; AppInputs_RequestImu(false);
    }
    if (trial.source != TRIAL_OFF && trial_epoch != stop_epoch)
        Trial_Start(&trial,TRIAL_OFF,0,HAL_GetTick());
    ServiceRetryHealth(control_release_ms);
    ServiceRx();                 /* STOP before another control output */
    EncoderService_Tick(HAL_GetTick()); /* observation only; never changes targets */
    ServiceTrial(HAL_GetTick());
    ServiceLive();
    ServicePosition(control_release_ms);
    ServiceDualPosition(control_release_ms);
    ReportPendingEvent();
    ServiceTelemetry(HAL_GetTick());
    uint32_t now=HAL_GetTick();
    bool active=live_mode && team_mode && live_epoch==stop_epoch;
    TeamLinkStatus team_status={
        .active=active,.permit=active && live_permit,
        .armed=active && live_control.armed,
        .config_ready=live_cfg.enable && Sy_Validate(&live_cfg),
        .radio_fresh=active && live_input.radio_ok && now-live_input.radio_ms<=300,
        .imu_fresh=active && live_input.imu_ok && now-live_input.imu_ms<=100,
        .sensors_valid=active && live_input.adc_ok,
        .mode=active?live_control.mode:SY_OFF,
        .fault=active?live_control.fault:SY_OK,
        .updated_ms=now
    };
    team_status.ready_to_arm=active && !live_permit && !fault_latched &&
        live_control.fault==SY_OK && team_status.config_ready &&
        team_status.radio_fresh && team_status.sensors_valid &&
        (!live_cfg.angle_enable || team_status.imu_fresh) &&
        live_input.requested==SY_HOVER && fabsf(live_input.stick)<=.05f;
    if (team_status.permit && team_status.armed && live_control.fault==SY_OK) {
        team_status.pwm[0]=live_control.pwm[0];team_status.pwm[1]=live_control.pwm[1];
    }
    TeamLink_UpdateStatus(&team_status);
    RecordMax(&stats.max_task_us, RuntimePort_ElapsedUs(started));
}
