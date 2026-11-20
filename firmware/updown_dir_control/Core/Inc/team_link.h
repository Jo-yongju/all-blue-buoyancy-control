#ifndef TEAM_LINK_H
#define TEAM_LINK_H
#include "app_inputs.h"
#include "team_input.h"
typedef struct {
    ReceiverCommand radio;
    AppImuSnapshot imu;
    bool have_radio;
} TeamLinkSnapshot;
typedef struct {
    bool active, permit, armed, ready_to_arm;
    bool config_ready, radio_fresh, imu_fresh, sensors_valid;
    SyMode mode;
    SyFault fault;
    float pwm[2]; /* controller request, not a hardware measurement */
    uint32_t updated_ms;
} TeamLinkStatus;
/* Call from team receiver/IMU TASKS on NEW validated frames/reports only.
 * Timestamps must use the same millisecond clock as ControlTask.
 * No motor writes, peripheral initialization or blocking calls here. */
void TeamLink_PublishRadio(uint16_t lh,uint16_t swc,bool failsafe,uint32_t received_ms);
void TeamLink_PublishPitch(float nose_up_deg,bool valid,uint32_t sampled_ms);
TeamLinkSnapshot TeamLink_Read(void);
void TeamLink_Clear(void);
/* Task context only, AFTER AppTransport_Init. These enqueue requests, not
 * immediate state changes. Check bool, then STATUS before requesting ARM. */
bool TeamLink_RequestObserve(void);
bool TeamLink_RequestArm(void);
/* Last ControlTask status, refreshed every 10ms. For startup/telemetry only;
 * never use this snapshot as another task's permission to write a motor. */
TeamLinkStatus TeamLink_ReadStatus(void);
void TeamLink_UpdateStatus(const TeamLinkStatus *status); /* ControlTask only */
#endif
