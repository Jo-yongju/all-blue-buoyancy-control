#include "main.h"
#include "trial_receiver.h"

static UART_HandleTypeDef uart;

bool TrialReceiver_Init(void)
{
    if (uart.Instance==USART1) return true;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef pin={.Pin=GPIO_PIN_10,.Mode=GPIO_MODE_AF_PP,.Pull=GPIO_PULLUP,
        .Speed=GPIO_SPEED_FREQ_LOW,.Alternate=GPIO_AF7_USART1};
    HAL_GPIO_Init(GPIOA,&pin);
    uart.Instance=USART1;
    uart.Init.BaudRate=115200;
    uart.Init.WordLength=UART_WORDLENGTH_8B;
    uart.Init.StopBits=UART_STOPBITS_1;
    uart.Init.Parity=UART_PARITY_NONE;
    uart.Init.Mode=UART_MODE_RX;
    uart.Init.OverSampling=UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart)!=HAL_OK) {uart.Instance=NULL;return false;}
    HAL_NVIC_SetPriority(USART1_IRQn,7,0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR3|=USART_CR3_EIE;
    USART1->CR1|=USART_CR1_RXNEIE_RXFNEIE;
    return true;
}
void USART1_IRQHandler(void)
{
    uint32_t flags=USART1->ISR;
    if (flags & (USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE|USART_ISR_PE)) {
        USART1->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_PECF;
        USART1->RQR=USART_RQR_RXFRQ;
        TrialReceiver_ErrorFromISR();
        return;
    }
    if (flags&USART_ISR_RXNE_RXFNE)
        TrialReceiver_PushFromISR((uint8_t)USART1->RDR,HAL_GetTick());
}
