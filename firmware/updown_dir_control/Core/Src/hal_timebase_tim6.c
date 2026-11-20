#include "main.h"
#include "motor_test.h"
/* TIM6 owns the HAL millisecond clock and motor stop checks, at IRQ priority 4.
 * It never calls FreeRTOS APIs. SysTick/PendSV belong exclusively to FreeRTOS. */
HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    (void)priority;
    __HAL_RCC_TIM6_CLK_ENABLE();
    uint32_t timer_hz=HAL_RCC_GetPCLK1Freq();
    if(RCC->CFGR & RCC_CFGR_PPRE1) timer_hz*=2;
    if(timer_hz<1000000U || timer_hz%1000000U) return HAL_ERROR;
    TIM6->CR1=0;
    TIM6->PSC=timer_hz/1000000U-1U;
    TIM6->ARR=999U;
    TIM6->EGR=TIM_EGR_UG;
    TIM6->SR=0;
    TIM6->DIER=TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,4,0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    uwTickPrio=4;
    TIM6->CR1=TIM_CR1_CEN;
    return HAL_OK;
}
void HAL_SuspendTick(void) { TIM6->DIER &= ~TIM_DIER_UIE; }
void HAL_ResumeTick(void) { TIM6->DIER |= TIM_DIER_UIE; }
void TIM6_DAC_IRQHandler(void)
{
    if(TIM6->SR & TIM_SR_UIF) {
        TIM6->SR=0;
        HAL_IncTick();
        MotorTest_Tick(HAL_GetTick());
    }
}
