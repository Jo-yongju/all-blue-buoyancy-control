#include "main.h"
#include "encoder_port.h"

static volatile EncoderRaw raw[2];
static bool active;
static const IRQn_Type irqs[4]={EXTI0_IRQn,EXTI1_IRQn,EXTI2_IRQn,EXTI3_IRQn};
static unsigned State(unsigned axis)
{
    uint32_t bits=(GPIOC->IDR>>(2U*axis))&3U;
    return ((bits&1U)<<1U)|((bits>>1U)&1U); /* A is high bit. */
}
void EncoderPort_Stop(void)
{
    for(unsigned i=0;i<4;i++) HAL_NVIC_DisableIRQ(irqs[i]);
    active=false;
}
void EncoderPort_Start(void)
{
    EncoderPort_Stop();
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_SYSCFG_CLK_ENABLE();
    GPIO_InitTypeDef gpio={0};
    gpio.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    gpio.Mode=GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull=GPIO_NOPULL; /* External encoder interface must supply valid levels. */
    HAL_GPIO_Init(GPIOC,&gpio);
    __HAL_GPIO_EXTI_CLEAR_IT(gpio.Pin);
    EncoderRaw_Init(&raw[0],State(0)); EncoderRaw_Init(&raw[1],State(1));
    active=true;
    for(unsigned i=0;i<4;i++) {
        HAL_NVIC_ClearPendingIRQ(irqs[i]);
        HAL_NVIC_SetPriority(irqs[i],6,0); HAL_NVIC_EnableIRQ(irqs[i]);
    }
}
EncoderRaw EncoderPort_Read(unsigned axis)
{
    EncoderRaw result={0};
    if(axis>=2) return result;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    result.count=raw[axis].count; result.errors=raw[axis].errors; result.ab=raw[axis].ab;
    if(!mask) __enable_irq();
    return result;
}
static void Edge(unsigned axis,uint16_t pin)
{
    if(__HAL_GPIO_EXTI_GET_IT(pin)!=0) {
        __HAL_GPIO_EXTI_CLEAR_IT(pin);
        if(active) EncoderRaw_Edge(&raw[axis],State(axis));
    }
}
/* Capture edges independently of task scheduling. No HAL delays, UART, or
 * RTOS calls here. Both edges lost during masking cannot always be detected. */
void EXTI0_IRQHandler(void){Edge(0,GPIO_PIN_0);}
void EXTI1_IRQHandler(void){Edge(0,GPIO_PIN_1);}
void EXTI2_IRQHandler(void){Edge(1,GPIO_PIN_2);}
void EXTI3_IRQHandler(void){Edge(1,GPIO_PIN_3);}
