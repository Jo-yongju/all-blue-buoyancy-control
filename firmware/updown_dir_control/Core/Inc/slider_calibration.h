#ifndef SLIDER_CALIBRATION_H
#define SLIDER_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

/* Points are ordered by physical position, NOT by ADC magnitude.
 * Both increasing and decreasing potentiometer wiring are supported. */
typedef struct {
    uint16_t adc[3];
    float mm[3];
    bool confirmed;
} SliderCalibration;

bool SliderCalibration_Valid(const SliderCalibration *cal);
bool SliderCalibration_Position(const SliderCalibration *cal, uint16_t raw, float *mm);
#endif
