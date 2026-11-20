#include "main.h"
#include "live_io.h"
#include "live_config.h"
#include "BNO080.h"
#include "runtime_policy.h"
#include "Quaternion.h"
#include <math.h>

bool LiveIMU_Init(void)
{
    if (BNO080_Initialization()!=0) return false;
    BNO080_enableRotationVector(IMU_REPORT_US);
    return true; /* A fresh valid rotation report is still required before ARM. */
}
bool LiveIMU_Sample(float *pitch)
{
    if (!BNO080_dataAvailable()) return false;
    float q[]={BNO080_getQuatI(),BNO080_getQuatJ(),BNO080_getQuatK(),BNO080_getQuatReal()};
    Quaternion_Update(q);
    *pitch=remainderf((BNO080_Pitch-LIVE_LEVEL_DEG)*LIVE_PITCH_SIGN,360.f);
    return true;
}

bool LiveIMU_Pending(void)
{
    return HAL_GPIO_ReadPin(BNO080_INT_PORT,BNO080_INT_PIN)==GPIO_PIN_RESET;
}
