#include "main.h"
#include "slider_sensor.h"
#include "runtime_policy.h"

static ADC_HandleTypeDef slider_adc;
static bool ready;

bool SliderSensor_Init(void)
{
    ready=false;
    RCC_PeriphCLKInitTypeDef clock={0};
    clock.PeriphClockSelection=RCC_PERIPHCLK_ADC12;
    clock.Adc12ClockSelection=RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK) return false;
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio={0};
    gpio.Pin=GPIO_PIN_0|GPIO_PIN_1;
    gpio.Mode=GPIO_MODE_ANALOG;
    gpio.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA,&gpio);

    slider_adc.Instance=ADC1;
    /* SYSCLK 170 MHz / 16 = 10.625 MHz. Long acquisition for a potentiometer. */
    slider_adc.Init.ClockPrescaler=ADC_CLOCK_ASYNC_DIV16;
    slider_adc.Init.Resolution=ADC_RESOLUTION_12B;
    slider_adc.Init.DataAlign=ADC_DATAALIGN_RIGHT;
    slider_adc.Init.GainCompensation=0;
    slider_adc.Init.ScanConvMode=ADC_SCAN_DISABLE;
    slider_adc.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
    slider_adc.Init.LowPowerAutoWait=DISABLE;
    slider_adc.Init.ContinuousConvMode=DISABLE;
    slider_adc.Init.NbrOfConversion=1;
    slider_adc.Init.DiscontinuousConvMode=DISABLE;
    slider_adc.Init.ExternalTrigConv=ADC_SOFTWARE_START;
    slider_adc.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;
    slider_adc.Init.DMAContinuousRequests=DISABLE;
    slider_adc.Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;
    slider_adc.Init.OversamplingMode=DISABLE;
    slider_adc.Init.SamplingMode=ADC_SAMPLING_MODE_NORMAL;
    if (HAL_ADC_Init(&slider_adc)!=HAL_OK) return false;

    ADC_MultiModeTypeDef multimode={0};
    multimode.Mode=ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&slider_adc,&multimode)!=HAL_OK) return false;
    ADC_ChannelConfTypeDef channel={0};
    channel.Channel=ADC_CHANNEL_1;
    channel.Rank=ADC_REGULAR_RANK_1;
    channel.SamplingTime=ADC_SAMPLETIME_640CYCLES_5;
    channel.SingleDiff=ADC_SINGLE_ENDED;
    channel.OffsetNumber=ADC_OFFSET_NONE;
    if (HAL_ADC_ConfigChannel(&slider_adc,&channel)!=HAL_OK) return false;
    if (HAL_ADCEx_Calibration_Start(&slider_adc,ADC_SINGLE_ENDED)!=HAL_OK) return false;
    ready=true;
    return true;
}

bool SliderSensor_Read(uint16_t *raw)
{
    return SliderSensor_ReadAxis(0,raw);
}

bool SliderSensor_ReadAxis(unsigned axis,uint16_t *raw)
{
    if (!ready || raw==NULL || axis>1) return false;
    ADC_ChannelConfTypeDef ch={.Channel=axis?ADC_CHANNEL_2:ADC_CHANNEL_1,
        .Rank=ADC_REGULAR_RANK_1,.SamplingTime=ADC_SAMPLETIME_640CYCLES_5,
        .SingleDiff=ADC_SINGLE_ENDED,.OffsetNumber=ADC_OFFSET_NONE};
    if (HAL_ADC_ConfigChannel(&slider_adc,&ch)!=HAL_OK) return false;
    uint32_t sum=0;
    uint32_t started = HAL_GetTick();
    for (unsigned i=0;i<8;i++) {
        uint32_t elapsed = HAL_GetTick() - started;
        if (elapsed >= SENSOR_TOTAL_BUDGET_MS) return false;
        if (HAL_ADC_Start(&slider_adc)!=HAL_OK) return false;
        if (HAL_ADC_PollForConversion(&slider_adc, SENSOR_TOTAL_BUDGET_MS - elapsed)!=HAL_OK) {
            (void)HAL_ADC_Stop(&slider_adc);
            return false;
        }
        sum+=HAL_ADC_GetValue(&slider_adc);
        if (HAL_ADC_Stop(&slider_adc)!=HAL_OK) return false;
    }
    if ((uint32_t)(HAL_GetTick() - started) >= SENSOR_TOTAL_BUDGET_MS) return false;
    *raw=(uint16_t)((sum+4U)/8U);
    return true;
}
