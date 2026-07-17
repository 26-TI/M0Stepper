/**
 * @file gyro.h
 * @brief WT901/JY901 串口陀螺仪协议解析（纯数据层，零硬件依赖）
 */

#ifndef __GYRO_H__
#define __GYRO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float wz;     ///< Z 轴角速度 (°/s)，±2000°/s
    float yaw;    ///< Yaw 角度 (°)，-180° ~ 180°
} GyroData;

/** 初始化解析器状态 */
void gyro_init(void);

/** 喂入一个接收字节，内部自动拼帧、校验、解析 */
void gyro_feed_byte(uint8_t byte);

/** 有新数据可用时返回 1（注意：调用 gyro_get_data 后会清标记） */
int  gyro_has_new_data(void);

/** 获取最新解析结果 */
GyroData gyro_get_data(void);

#ifdef __cplusplus
}
#endif

#endif /* __GYRO_H__ */
