/**
 * @file bsp_can.h
 * @brief BSP CAN 抽象 — 中断接收 + 查询发送
 *
 * 换 MCU 时只需重写 bsp_can.cpp，接口保持不变
 * PA12=CAN_TX, PA13=CAN_RX, 250kbps, CAN 2.0 标准帧
 */

#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_can_init(void);
bool bsp_can_send(uint32_t id, uint8_t *data, uint8_t len);
bool bsp_can_recv(uint32_t *id, uint8_t *data, uint8_t *len);
bool bsp_can_has_msg(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H__ */
