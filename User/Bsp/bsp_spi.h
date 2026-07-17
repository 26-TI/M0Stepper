/**
 * @file bsp_spi.h
 * @brief BSP SPI 抽象 — 全双工单字节传输
 *
 * 换 MCU 时只需重写 bsp_spi.cpp，接口保持不变
 */

#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void    bsp_spi_init(void);
uint8_t bsp_spi_transfer(uint8_t tx);  ///< 全双工单字节（发 tx 同时收 rx）

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H__ */
