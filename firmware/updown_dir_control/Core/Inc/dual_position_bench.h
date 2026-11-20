#ifndef DUAL_POSITION_BENCH_H
#define DUAL_POSITION_BENCH_H
#include "position_control.h"
/* ControlTask owns this state. This PC fixture does not enable IMU/RC modes. */
typedef struct {
    PositionConfig config[2];
    PositionControl axis[2];
    unsigned fault_axis; /* 1 or 2; zero means neither */
} DualPositionBench;
void DualPosition_Init(DualPositionBench *s);
PositionStatus DualPosition_Start(DualPositionBench *s,unsigned target,
    const uint16_t raw[2],uint32_t now);
PositionStatus DualPosition_Update(DualPositionBench *s,const bool ok[2],
    const uint16_t raw[2],uint32_t now);
#endif
