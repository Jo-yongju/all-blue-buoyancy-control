#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"
#include "app_transport.h"
#include "app_inputs.h"
#include "trial_receiver.h"
#include "control_app.h"
#include "motor_test.h"
#include "runtime_port.h"
#include "runtime_policy.h"
#include <stdio.h>

/* Static stacks in words: 4 KiB control, 2 KiB receiver, 3 KiB IMU/console. */
static StaticTask_t control_tcb,receiver_tcb,imu_tcb,console_tcb,idle_tcb;
static StackType_t control_stack[1024],receiver_stack[512],imu_stack[768],console_stack[768];
static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
static TaskHandle_t control_handle,receiver_handle,imu_handle,console_handle;
typedef struct { uint32_t runs,max_us,backlogs; } ServiceStats;
static volatile ServiceStats rx_stats,imu_stats;
static struct { uint32_t cycles, max_us, max_gap_ms, late_ms, overruns; } timing;
/* Inspect these in the debugger after an assert/stack failure. */
volatile const char *app_fatal_file;
volatile unsigned app_fatal_line;

static void ControlTask(void *argument)
{
    (void)argument;
    TickType_t release=xTaskGetTickCount();
    uint32_t previous=HAL_GetTick();
    for (;;) {
        TickType_t now_tick=xTaskGetTickCount();
        uint32_t lateness=(uint32_t)(now_tick-release);
        uint32_t now=HAL_GetTick(), gap=now-previous;
        previous=now;
        if(gap>timing.max_gap_ms) timing.max_gap_ms=gap;
        if(lateness>timing.late_ms) timing.late_ms=lateness;
        if(lateness>=CONTROL_PERIOD_MS) {
            MotorTest_Stop(); ++timing.overruns;
            release=now_tick; /* Skip stale releases; never run a catch-up burst. */
        }
        uint32_t cycles=RuntimePort_Cycles();
        ControlApp_Step();
        uint32_t elapsed=RuntimePort_ElapsedUs(cycles);
        ++timing.cycles;
        if(elapsed>timing.max_us) timing.max_us=elapsed;
        if(elapsed>=CONTROL_PERIOD_MS*1000U) {
            MotorTest_Stop(); ++timing.overruns;
            release=xTaskGetTickCount();
        }
        vTaskDelayUntil(&release,pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
static void RecordService(volatile ServiceStats *s,uint32_t start,bool backlog)
{
    uint32_t elapsed=RuntimePort_ElapsedUs(start);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    ++s->runs; if(elapsed>s->max_us) s->max_us=elapsed;
    if(backlog) ++s->backlogs;
    if(!mask) __enable_irq();
}
static void ReceiverTask(void *argument)
{
    (void)argument;
    for (;;) {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        bool more;
        do {
            uint32_t start=RuntimePort_Cycles();
            more=TrialReceiver_Step();
            RecordService(&rx_stats,start,more);
            /* Bounded batches even under a continuous/noisy UART stream. */
            if(more) vTaskDelay(pdMS_TO_TICKS(INPUT_BACKLOG_YIELD_MS));
        } while(more);
    }
}
static void ImuTask(void *argument)
{
    (void)argument;
    bool more=false;
    for (;;) {
        if(more) vTaskDelay(pdMS_TO_TICKS(INPUT_BACKLOG_YIELD_MS));
        else ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(IMU_EVENT_CHECK_MS));
        uint32_t start=RuntimePort_Cycles();
        for(unsigned n=0;n<IMU_PACKET_BUDGET;n++) {
            AppInputs_Step();
            more=AppInputs_ImuPending();
            if(!more) break;
        }
        RecordService(&imu_stats,start,more);
        /* Level check drains packets when INT stays low; timeout catches a lost
         * edge. Neither is a fixed 400 Hz task release. Boot may take longer. */
    }
}
static void NotifyFromISR(TaskHandle_t handle)
{
    BaseType_t wake=pdFALSE;
    if(handle) vTaskNotifyGiveFromISR(handle,&wake);
    portYIELD_FROM_ISR(wake);
}
void AppTasks_NotifyReceiverFromISR(void) { NotifyFromISR(receiver_handle); }
void AppTasks_NotifyImuFromISR(void) { NotifyFromISR(imu_handle); }
void AppTasks_NotifyImu(void) { if(imu_handle) xTaskNotifyGive(imu_handle); }
static void ConsoleTask(void *argument)
{
    (void)argument;
    for (;;) {
        ControlApp_ConsoleStep();
        vTaskDelay(pdMS_TO_TICKS(CONSOLE_SERVICE_MS));
    }
}
void AppTasks_Start(void)
{
    AppTransport_Init();
    control_handle=xTaskCreateStatic(ControlTask,"control",1024,NULL,CONTROL_PRIORITY,control_stack,&control_tcb);
    receiver_handle=xTaskCreateStatic(ReceiverTask,"receiver",512,NULL,RECEIVER_PRIORITY,receiver_stack,&receiver_tcb);
    imu_handle=xTaskCreateStatic(ImuTask,"imu",768,NULL,IMU_PRIORITY,imu_stack,&imu_tcb);
    console_handle=xTaskCreateStatic(ConsoleTask,"console",768,NULL,CONSOLE_PRIORITY,console_stack,&console_tcb);
    configASSERT(control_handle && receiver_handle && imu_handle && console_handle);
    vTaskStartScheduler();
    AppTasks_Fatal(__FILE__,__LINE__);
}
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,StackType_t **stack,uint32_t *size)
{
    *tcb=&idle_tcb; *stack=idle_stack; *size=configMINIMAL_STACK_SIZE;
}
void vApplicationStackOverflowHook(TaskHandle_t task,char *name)
{
    (void)task; AppTasks_Fatal(name,0);
}
void AppTasks_Fatal(const char *file,unsigned line)
{
    __disable_irq(); MotorTest_Stop();
    app_fatal_file=file; app_fatal_line=line;
    for (;;) __NOP();
}
void AppTasks_DelayMs(uint32_t milliseconds)
{
    if(xTaskGetSchedulerState()==taskSCHEDULER_RUNNING)
        vTaskDelay(pdMS_TO_TICKS(milliseconds)+1U);
    else HAL_Delay(milliseconds);
}
void AppTasks_FormatStats(char *text,size_t size)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    ServiceStats rx=rx_stats,im=imu_stats;
    if(!mask) __enable_irq();
    ReceiverStats r=TrialReceiver_Stats();
    /* ControlTask formats; console transmits without blocking control. */
    snprintf(text,size,
        "RTOS pri=4,3,2,1 control_ms=%u receiver=EVENT imu=EVENT report_us=%u check_ms=%u console_ms=%u "
        "gap_max_ms=%lu late_max_ms=%lu work_max_us=%lu overruns=%lu "
        "rx_runs=%lu rx_max_us=%lu rx_backlogs=%lu frames=%lu rx_overflow=%lu rx_errors=%lu rx_peak=%lu "
        "imu_runs=%lu imu_max_us=%lu imu_backlogs=%lu stack_free_words=%lu,%lu,%lu,%lu (control,receiver,imu,console)\r\n",
        CONTROL_PERIOD_MS,IMU_REPORT_US,IMU_EVENT_CHECK_MS,CONSOLE_SERVICE_MS,
        (unsigned long)timing.max_gap_ms,(unsigned long)timing.late_ms,
        (unsigned long)timing.max_us,(unsigned long)timing.overruns,
        (unsigned long)rx.runs,(unsigned long)rx.max_us,(unsigned long)rx.backlogs,
        (unsigned long)r.frames,(unsigned long)r.overflow,(unsigned long)r.uart_errors,(unsigned long)r.peak_bytes,
        (unsigned long)im.runs,(unsigned long)im.max_us,(unsigned long)im.backlogs,
        (unsigned long)uxTaskGetStackHighWaterMark(control_handle),
        (unsigned long)uxTaskGetStackHighWaterMark(receiver_handle),
        (unsigned long)uxTaskGetStackHighWaterMark(imu_handle),
        (unsigned long)uxTaskGetStackHighWaterMark(console_handle));
}
