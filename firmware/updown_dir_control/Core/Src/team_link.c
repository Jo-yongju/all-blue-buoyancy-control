#include "main.h"
#include "team_link.h"
#include "app_transport.h"
#include "motor_test.h"
#include <math.h>
#include <string.h>

static TeamLinkSnapshot latest;
static TeamLinkStatus runtime;
void TeamLink_Clear(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    latest=(TeamLinkSnapshot){0};
    if (!mask) __enable_irq();
}
void TeamLink_PublishRadio(uint16_t lh,uint16_t swc,bool failsafe,uint32_t stamp)
{
    ReceiverCommand c=TeamInput_Radio(lh,swc,failsafe,stamp);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    /* A delayed publisher must not overwrite a newer report. Equal timestamps
     * are allowed because multiple frames can be processed in one ms. */
    if (!latest.have_radio || (int32_t)(stamp-latest.radio.received_ms)>=0) {
        latest.radio=c; latest.have_radio=true;
    }
    if (!mask) __enable_irq();
}
void TeamLink_PublishPitch(float pitch,bool valid,uint32_t stamp)
{
    AppImuSnapshot sample={.pitch_deg=pitch,.sampled_ms=stamp,
        .valid=valid && isfinite(pitch) && fabsf(pitch)<=180,.initialized=true};
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!latest.imu.initialized || (int32_t)(stamp-latest.imu.sampled_ms)>=0)
        latest.imu=sample;
    if (!mask) __enable_irq();
}
TeamLinkSnapshot TeamLink_Read(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    TeamLinkSnapshot copy=latest;
    if (!mask) __enable_irq();
    return copy;
}
static bool request(const char *line)
{
    AppCommandMessage message={.epoch=MotorTest_Generation()};
    strcpy(message.line,line);
    return AppTransport_Send(&message);
}
bool TeamLink_RequestObserve(void) { return request("MODE TEAM"); }
bool TeamLink_RequestArm(void) { return request("LIVE ARM"); }
TeamLinkStatus TeamLink_ReadStatus(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    TeamLinkStatus copy=runtime;
    if (!mask) __enable_irq();
    return copy;
}
void TeamLink_UpdateStatus(const TeamLinkStatus *status)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    runtime=*status;
    if (!mask) __enable_irq();
}
