/**
 * @file bsp_imu601.h
 * @brief 汇电籽-601 (ICM42688) UART 通信层（Bsp 层，MCU 相关）
 *
 * 唯一依赖 ti_msp_dl_config.h 的层。换 MCU 时需要重写整个 .cpp。
 *
 * === 硬件 ===
 *
 *   UART2, PA22(RX), PA21(TX), 115200 8N1
 *
 * === SysConfig 配置 ===
 *
 *   Name:      IMU601
 *   Instance:  UART2
 *   RX Pin:    PA22
 *   TX Pin:    PA21
 *   Baud:      115200
 *   RX 中断:   开启
 *
 * === 接收策略 ===
 *
 *   ISR + Polling 双路径：
 *   - ISR（UART2_IRQHandler）：RX FIFO ≥4 字节时触发，数据进 ring buffer
 *   - Polling（bsp_imu601_poll 内）：每 1ms 直接读 RX FIFO 兜底
 *
 *   为什么需要 polling 兜底：
 *   模块低速率（~240 bytes/200ms），主循环 1ms 一次比 FIFO 阈值（4 字节）
 *   更频繁，FIFO 攒不够触发量 → ISR 几乎不响。polling 主动捞数据，
 *   不管 ISR 有没有触发。
 *
 * === 上电初始化 ===
 *
 *   bsp_imu601_init() 会自动发一条 AA 55 软复位指令，模块重启后立即
 *   开始持续发送姿态数据。不发送校准指令（参考例程的 float 参数
 *   可能与你的模块不兼容，会导致模块死机）。
 *
 * === 手动复位 ===
 *
 *   需要时调用 bsp_imu601_reset()，比如模块卡死后恢复。
 */

#ifndef __BSP_IMU601_H__
#define __BSP_IMU601_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化：软复位 + 开 RX 中断 + 屏蔽错误中断 */
void bsp_imu601_init(void);

/** 手动软复位（模块卡死时调用） */
void bsp_imu601_reset(void);

/** 主循环每 1ms 调用：消费 ring buffer + 兜底 polling */
void bsp_imu601_poll(void);

float    bsp_imu601_get_yaw(void);
float    bsp_imu601_get_pitch(void);
float    bsp_imu601_get_roll(void);
uint32_t bsp_imu601_rx_total(void);   /* 累计收字节数，调试用 */

#ifdef __cplusplus
}
#endif

#endif
