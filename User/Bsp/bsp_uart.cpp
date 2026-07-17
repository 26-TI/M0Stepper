/**
 * @file bsp_uart.cpp
 * @brief BSP UART — MSPM0 UART0 printf 调试输出
 *
 * 天猛星烧录口串口，阻塞式发送
 */

#include "bsp_uart.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdarg.h>

/**
 * UART0 RX ISR — 只吃掉收到的字节，不缓存
 * 防止 PC 串口助手回发数据时触发 Default_Handler 死循环
 */
extern "C" void UART0_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_DEBUG_INST))
    {
        case DL_UART_IIDX_RX:
        {
            /* 排空 RX FIFO（上限 32 次防死循环） */
            int limit = 32;
            while (limit-- > 0 && !DL_UART_isRXFIFOEmpty(UART_DEBUG_INST))
            {
                DL_UART_receiveData(UART_DEBUG_INST);
            }
            break;
        }
        default:
            break;
    }
}

int bsp_uart_printf(const char *fmt, ...)
{
    static char buf[128];
    uint32_t    i, len;
    va_list     args;
    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (i = 0; i < len; i++)
    {
        DL_UART_transmitData(UART_DEBUG_INST, buf[i]);
        while (!DL_UART_isTXFIFOEmpty(UART0))
            ;
    }
    return len;
}
