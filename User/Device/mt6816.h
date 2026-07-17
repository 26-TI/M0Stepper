/**
 * @file mt6816.h
 * @brief MT6816 14-bit 磁编码器驱动（器件层）
 *
 * 依赖：bsp_spi.h / bsp_gpio.h
 * 与 MCU 无关，换平台只需 Bsp 层实现同名函数
 */

#ifndef __MT6816_H__
#define __MT6816_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 编码器数据结构体 */
typedef struct
{
    uint8_t mag;        ///< 磁铁检测标志：0=正常, 1=异常
    uint8_t raw_h;      ///< SPI 读回的原始高字节
    uint8_t raw_l;      ///< SPI 读回的原始低字节
    int     raw_angle;  ///< 拼接后的 14 位原始角度值（0~16383）
    float   angle;      ///< 角度制（0°~360°）
} MT6816_Data;

void mt6816_init(void);                       ///< 初始化：CS 拉高，结构体清零
void mt6816_read(MT6816_Data *data);          ///< 读一次角度数据

#ifdef __cplusplus
}
#endif

#endif /* __MT6816_H__ */
