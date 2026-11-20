#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct {uint32_t APB1ENR1,AHB2ENR,CFGR;} RCC_Type;
typedef struct {uint32_t CCR1,CCR4,CCMR2,CCER,PSC,ARR,CCMR1,EGR,SR,CR1;} TIM_Type;
typedef struct {uint32_t BSRR;} GPIO_Type;
typedef struct {uint32_t ISR;} USART_Type;
typedef struct {USART_Type *Instance;} UART_HandleTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
extern RCC_Type test_rcc;extern TIM_Type test_tim;extern GPIO_Type test_gpioa,test_gpiob;
extern UART_HandleTypeDef hcom_uart[1];
#define RCC (&test_rcc)
#define TIM3 (&test_tim)
#define GPIOA (&test_gpioa)
#define GPIOB (&test_gpiob)
#define RCC_APB1ENR1_TIM3EN 1u
#define RCC_AHB2ENR_GPIOBEN 2u
#define RCC_CFGR_PPRE1 0x700u
#define TIM_CCER_CC1E 1u
#define GPIO_PIN_0 1u
#define GPIO_PIN_1 2u
#define GPIO_PIN_6 64u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_AF_PP 2u
#define GPIO_PULLDOWN 2u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_AF2_TIM3 2u
#define TIM_CCMR1_OC1M_1 0x20u
#define TIM_CCMR1_OC1M_2 0x40u
#define TIM_EGR_UG 1u
#define TIM_CR1_CEN 1u
#define USART_ISR_ORE 1u
#define USART_ISR_FE 2u
#define USART_ISR_NE 4u
#define USART_ISR_PE 8u
#define UART_CLEAR_OREF 1u
#define UART_CLEAR_FEF 2u
#define UART_CLEAR_NEF 4u
#define UART_CLEAR_PEF 8u
#define UART_RXDATA_FLUSH_REQUEST 1u
#define COM1 0
#define HAL_OK 0
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() (RCC->AHB2ENR|=2u)
#define __HAL_RCC_TIM3_CLK_ENABLE() (RCC->APB1ENR1|=1u)
#define __HAL_RCC_TIM3_FORCE_RESET() ((void)0)
#define __HAL_RCC_TIM3_RELEASE_RESET() ((void)0)
#define __HAL_DBGMCU_FREEZE_TIM3() ((void)0)
#define __HAL_UART_CLEAR_FLAG(u,f) ((u)->Instance->ISR&=~(f))
#define __HAL_UART_SEND_REQ(u,f) ((void)(u),(void)(f))
#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_GPIO_Init(GPIO_Type *,GPIO_InitTypeDef *);
void Error_Handler(void);
int HAL_UART_Receive(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
int HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
extern TIM_Type test_tim4;
extern GPIO_Type test_gpioc;
#define TIM4 (&test_tim4)
#define GPIOC (&test_gpioc)
#define RCC_APB1ENR1_TIM4EN 4u
#define GPIO_PIN_4 16u
#define GPIO_PIN_7 128u
#define GPIO_PIN_9 512u
#define GPIO_AF2_TIM4 2u
#define TIM_CCER_CC4E 4096u
#define TIM_CCMR2_OC4M_1 0x2000u
#define TIM_CCMR2_OC4M_2 0x4000u
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM4_CLK_ENABLE() (RCC->APB1ENR1|=4u)
#define __HAL_RCC_TIM4_FORCE_RESET() ((void)0)
#define __HAL_RCC_TIM4_RELEASE_RESET() ((void)0)
#define __HAL_DBGMCU_FREEZE_TIM4() ((void)0)
#endif
