/********************************** (C) COPYRIGHT *******************************
* File Name          : comm.c
* Description        : USART3 console, RX interrupt with a lock free FIFO.
*******************************************************************************/

#include "comm.h"
#include <stddef.h>
#include <string.h>

#define COMM_RX_FIFO_SIZE   256U    /* must stay a power of two */

static volatile uint8_t  s_fifo[COMM_RX_FIFO_SIZE];
static volatile uint16_t s_head;    /* written by the ISR  */
static volatile uint16_t s_tail;    /* written by the task */
static char              s_line[COMM_LINE_MAX];
static uint16_t          s_len;
static volatile uint32_t s_overflow;

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void COMM_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  gpio = {0};
    USART_InitTypeDef uart = {0};
    NVIC_InitTypeDef  nvic = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_10;            /* TX */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;            /* RX */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    uart.USART_BaudRate = baudrate;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &uart);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART3, ENABLE);

    s_head = 0U;
    s_tail = 0U;
    s_len = 0U;
    s_overflow = 0U;
}

/*
 * Pushes received bytes into the FIFO. Deliberately free of FreeRTOS calls:
 * the interrupt stays independent from the scheduler.
 */
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t  byte = (uint8_t)USART_ReceiveData(USART3);
        uint16_t next = (uint16_t)((s_head + 1U) & (COMM_RX_FIFO_SIZE - 1U));

        if(next == s_tail)
        {
            s_overflow++;
        }
        else
        {
            s_fifo[s_head] = byte;
            s_head = next;
        }
    }

    /* Clear a stale overrun so the receiver keeps working. */
    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART3);
    }
}

uint8_t COMM_PollLine(char *out, uint16_t max_len)
{
    if((out == NULL) || (max_len == 0U))
    {
        return 0U;
    }

    while(s_tail != s_head)
    {
        char c = (char)s_fifo[s_tail];

        s_tail = (uint16_t)((s_tail + 1U) & (COMM_RX_FIFO_SIZE - 1U));

        if((c == '\r') || (c == '\n'))
        {
            if(s_len > 0U)
            {
                s_line[s_len] = '\0';
                strncpy(out, s_line, (size_t)(max_len - 1U));
                out[max_len - 1U] = '\0';
                s_len = 0U;
                return 1U;
            }
            continue;
        }

        if((c == 0x08) || (c == 0x7F))
        {
            if(s_len > 0U)
            {
                s_len--;
            }
            continue;
        }

        if(s_len < (COMM_LINE_MAX - 1U))
        {
            s_line[s_len++] = c;
        }
        else
        {
            s_len = 0U;     /* overlong line: drop it and hunt for the next CR/LF */
        }
    }

    return 0U;
}

uint32_t COMM_GetOverflowCount(void)
{
    return s_overflow;
}

void COMM_ResetOverflowCount(void)
{
    s_overflow = 0U;
}
