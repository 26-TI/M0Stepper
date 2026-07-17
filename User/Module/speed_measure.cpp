/**
 * @file speed_measure.cpp
 * @brief 编码器测速 — 10ms 窗口角度差分 + 0.5 低通滤波
 */

#include "speed_measure.h"

void speed_measure_init(SpeedMeasure *sm)
{
    sm->actual_rpm     = 0.0f;
    sm->last_angle     = 0.0f;
    sm->speed_filtered = 0.0f;
    sm->first_read     = 1;
}

int speed_measure_update(SpeedMeasure *sm, MT6816_Data *enc)
{
    if (sm->first_read)
    {
        sm->last_angle = enc->angle;
        sm->first_read = 0;
        return 0;
    }

    static uint32_t tick = 0;
    static float    angle_10ms = 0.0f;
    tick++;

    if ((tick % 10) != 0)
        return 0;

    /* 10ms 角度差分 → RPM */
    float delta = enc->angle - angle_10ms;
    angle_10ms  = enc->angle;

    if (delta > 180.0f)  delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    float raw = -delta / 0.01f / 360.0f * 60.0f;
    sm->speed_filtered = 0.5f * raw + 0.5f * sm->speed_filtered;
    sm->actual_rpm = sm->speed_filtered;
    return 1;
}
