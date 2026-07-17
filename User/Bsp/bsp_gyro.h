/**
 * @file bsp_gyro.h
 * @brief WT901/JY901 串口陀螺仪 UART 通信层（UART1, PA18/PA17）
 *
 * 协议：0x5A 帧头，5 字节帧，校验和
 */

#ifndef __BSP_GYRO_H__
#define __BSP_GYRO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_gyro_init(void);
void bsp_gyro_poll(void);
float bsp_gyro_get_wz(void);
float bsp_gyro_get_yaw(void);
uint32_t bsp_gyro_rx_total(void);

#ifdef __cplusplus
}
#endif

#endif
