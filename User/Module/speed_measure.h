/**
 * @file speed_measure.h
 * @brief 编码器测速 — 10ms 窗口角度差分 + 低通滤波
 *
 * 纯测速模块，不包含 PID。需要 PID 请用 Module/pid.h
 */

#ifndef __SPEED_MEASURE_H__
#define __SPEED_MEASURE_H__

#include "mt6816.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 测速器 */
typedef struct
{
    float   actual_rpm;     ///< 当前速度 (RPM, 滤波后)
    float   last_angle;     ///< 上次角度
    float   speed_filtered; ///< 低通滤波值
    int     first_read;     ///< 首次标志
} SpeedMeasure;

void speed_measure_init(SpeedMeasure *sm);
int  speed_measure_update(SpeedMeasure *sm, MT6816_Data *enc);  ///< 返回 1=有新数据

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_MEASURE_H__ */
