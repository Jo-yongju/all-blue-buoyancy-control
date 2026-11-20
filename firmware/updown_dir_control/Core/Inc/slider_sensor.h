#ifndef SLIDER_SENSOR_H
#define SLIDER_SENSOR_H
#include <stdint.h>
#include <stdbool.h>

/* PA0 / CN7 pin 28: 0..VDDA only. No position calibration is inferred. */
bool SliderSensor_Init(void);
bool SliderSensor_Read(uint16_t *raw);
bool SliderSensor_ReadAxis(unsigned axis, uint16_t *raw);
#endif
