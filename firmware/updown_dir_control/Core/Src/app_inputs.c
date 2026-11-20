#include "main.h"
#include "app_inputs.h"
#include "app_tasks.h"
#include "live_io.h"
#include <math.h>
/* ImuTask alone touches the BNO driver. Short IRQ-masked copies protect
 * request/snapshot consistency. No lock is held during SPI or sensor boot. */
static volatile uint32_t request_generation;
static volatile bool requested;
static AppImuSnapshot latest;
static uint32_t handled_generation;
static bool driver_ready;
void AppInputs_RequestImu(bool enabled)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    requested=enabled; ++request_generation;
    latest=(AppImuSnapshot){.initializing=enabled};
    if (!mask) __enable_irq();
    AppTasks_NotifyImu();
}
AppImuSnapshot AppInputs_ReadImu(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    AppImuSnapshot copy=latest;
    if (!mask) __enable_irq();
    return copy;
}
void AppInputs_Step(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint32_t generation=request_generation;
    bool enabled=requested;
    if (!mask) __enable_irq();
    if (generation!=handled_generation) {
        driver_ready=enabled && LiveIMU_Init();
        handled_generation=generation;
        mask=__get_PRIMASK(); __disable_irq();
        if (generation==request_generation) {
            latest.initialized=driver_ready;
            latest.initializing=false;
        }
        if (!mask) __enable_irq();
    }
    if (!enabled || !driver_ready || generation!=request_generation) return;
    float pitch;
    if (!LiveIMU_Sample(&pitch)) return;
    AppImuSnapshot sample={.pitch_deg=pitch,.sampled_ms=HAL_GetTick(),
                          .valid=isfinite(pitch),.initialized=true};
    mask=__get_PRIMASK(); __disable_irq();
    if (generation==request_generation && requested) latest=sample;
    if (!mask) __enable_irq();
}

bool AppInputs_ImuPending(void)
{
    return requested && driver_ready && LiveIMU_Pending();
}
