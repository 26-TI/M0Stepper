/**
 * @file bsp_uart.h
 * @brief BSP UART 抽象 — printf 调试输出
 *
 * 换 MCU 时只需重写 bsp_uart.cpp，接口保持不变
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

int bsp_uart_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H__ */
