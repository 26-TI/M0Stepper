/**
 * @file bsp_i2c.h
 * @brief BSP I2C 抽象 — 主机发送
 *
 * 换 MCU 时只需重写 bsp_i2c.cpp，接口保持不变
 */

#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_i2c_send(uint8_t addr, const uint8_t *buf, uint8_t len);  ///< 主机发送

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H__ */
