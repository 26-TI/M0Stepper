/**
 * @file imu601.h
 * @brief 汇电籽-601 (ICM42688) 姿态模块 — 协议解析层（Device 层）
 *
 * 零硬件依赖，纯 C 状态机。换 MCU 不需任何修改。
 *
 * === 帧格式（12 字节定长）===
 *
 *   [0] 0xAA  [1] 0x55     帧头
 *   [2] 0x60               设备 ID（固定）
 *   [3] 0x01               指令码（0x01 = 姿态数据）
 *   [4] 0x06               数据长度（Payload 6 字节）
 *   [5..10]                Payload（小端，原始值 ×100）
 *     [5] [6]              Yaw   uint16  → ÷100.0 = 角度 °
 *     [7] [8]              Pitch int16   → ÷100.0 = 角度 °
 *     [9] [10]             Roll  int16   → ÷100.0 = 角度 °
 *   [11]                   校验和 = sum(bytes[2..10]) & 0xFF
 *
 * === 校验和算法 ===
 *
 *   cksum = buf[2] + buf[3] + ... + buf[10]
 *   跳过帧头 AA 55，不包含校验字节本身。
 *
 * === 精度 ===
 *
 *   0.01°（原始值 ×100，÷100 得浮点角度）
 *
 * === 上层调用方式 ===
 *
 *   imu601_init();
 *   每收到一个字节 → imu601_feed_byte(byte);
 *   if (imu601_has_new_data()) {
 *       IMU601_Data d = imu601_get_data();
 *       // d.yaw, d.pitch, d.roll
 *   }
 */

#ifndef __IMU601_H__
#define __IMU601_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float yaw;    ///< 偏航角 (°)，范围取决于模块配置
    float pitch;  ///< 俯仰角 (°)
    float roll;   ///< 横滚角 (°)
} IMU601_Data;

void imu601_init(void);
void imu601_feed_byte(uint8_t byte);
int  imu601_has_new_data(void);
IMU601_Data imu601_get_data(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU601_H__ */
