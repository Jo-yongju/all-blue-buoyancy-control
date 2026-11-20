#include "main.h"
#include "runtime_port.h"
#include "motor_test.h"
#include "receive_queue.h"

static bool clock_ready;
static volatile ReceiveQueue rx_queue;

void RuntimePort_StartReceiver(void)
{
    USART_TypeDef *uart = hcom_uart[COM1].Instance;
    /* NUCLEO-G431RB BSP COM1 is LPUART1, verified in the bundled BSP header.
     * Select its IRQ below rather than assuming the F407 USART2 mapping. */
    if (uart != LPUART1) { Error_Handler(); return; }
    ReceiveQueue_Clear(&rx_queue);
    uart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    uart->RQR = USART_RQR_RXFRQ;
    HAL_NVIC_SetPriority(LPUART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
    uart->CR3 |= USART_CR3_EIE;
    uart->CR1 |= USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE;
}

bool RuntimePort_ReadByte(uint8_t *byte)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool available = ReceiveQueue_Pop(&rx_queue, byte);
    if (!mask) __enable_irq();
    return available;
}

void RuntimePort_UartIrq(void)
{
    USART_TypeDef *uart = hcom_uart[COM1].Instance;
    for (unsigned i = 0; i < 16; ++i) {
        uint32_t flags = uart->ISR;
        if (flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
            MotorTest_CommunicationStop(true);
            ReceiveQueue_Clear(&rx_queue);
            uart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
            uart->RQR = USART_RQR_RXFRQ;
            return;
        }
        if (!(flags & USART_ISR_RXNE_RXFNE)) return;
        uint8_t byte = (uint8_t)uart->RDR;
        if (byte == '!') {
            MotorTest_CommunicationStop(false);
            ReceiveQueue_Clear(&rx_queue);
        } else if (!ReceiveQueue_Push(&rx_queue, byte)) {
            MotorTest_CommunicationStop(true);
            ReceiveQueue_Clear(&rx_queue);
            return;
        }
    }
}

bool RuntimePort_WriteByte(uint8_t byte)
{
    USART_TypeDef *uart = hcom_uart[COM1].Instance;
    if (uart == NULL || !(uart->ISR & USART_ISR_TXE_TXFNF)) return false;
    uart->TDR = byte;
    return true;
}

void RuntimePort_InitClock(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    for (volatile unsigned i = 0; i < 16; ++i) __NOP();
    clock_ready = DWT->CYCCNT != start && SystemCoreClock >= 1000000U;
}

bool RuntimePort_ClockReady(void) { return clock_ready; }
uint32_t RuntimePort_Cycles(void) { return DWT->CYCCNT; }

uint32_t RuntimePort_ElapsedUs(uint32_t since)
{
    if (!clock_ready) return 0;
    /* Differences only, < one counter wrap (~25s at 170MHz). Code regions
     * measured here have millisecond deadlines. Not a wall-clock timestamp. */
    return (DWT->CYCCNT - since) / (SystemCoreClock / 1000000U);
}
