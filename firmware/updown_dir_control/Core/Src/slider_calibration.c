#include "slider_calibration.h"
#include <math.h>
#include <stddef.h>

bool SliderCalibration_Valid(const SliderCalibration *cal)
{
    if (cal == NULL || !cal->confirmed) return false;
    for (unsigned i = 0; i < 3; ++i) {
        /* Full-scale endpoints are valid calibration observations. Runtime
         * travel limits and LIVE rail checks remain separate from conversion. */
        if (cal->adc[i] > 4095 || !isfinite(cal->mm[i])) return false;
    }
    int first = (int)cal->adc[1] - cal->adc[0];
    int second = (int)cal->adc[2] - cal->adc[1];
    bool increasing = first >= 50 && second >= 50;
    bool decreasing = first <= -50 && second <= -50;
    return (increasing || decreasing) && cal->mm[0] >= 0.0f &&
           cal->mm[1] > cal->mm[0] && cal->mm[2] > cal->mm[1];
}

bool SliderCalibration_Position(const SliderCalibration *cal, uint16_t raw, float *mm)
{
    if (mm == NULL || !SliderCalibration_Valid(cal)) return false;
    bool increasing = cal->adc[2] > cal->adc[0];
    if ((increasing && (raw < cal->adc[0] || raw > cal->adc[2])) ||
        (!increasing && (raw > cal->adc[0] || raw < cal->adc[2]))) return false;
    unsigned i = (increasing ? raw <= cal->adc[1] : raw >= cal->adc[1]) ? 0 : 1;
    float fraction = ((float)raw - cal->adc[i]) / ((float)cal->adc[i+1] - cal->adc[i]);
    *mm = cal->mm[i] + fraction * (cal->mm[i+1] - cal->mm[i]);
    return isfinite(*mm);
}
