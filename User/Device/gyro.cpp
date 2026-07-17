/**
 * @file gyro.cpp
 * @brief WT901/JY901 协议解析实现
 *
 * 帧格式（5 字节）：[0x5A, Type, DataL, DataH, Checksum]
 * Checksum = (0x5A + Type + DataL + DataH) 的低 8 位
 *
 * Type:
 *   0xAA — Z 轴角速度 wz，范围 ±2000°/s
 *   0xBB — Yaw 角度，范围 -180° ~ 180°
 */
#include "gyro.h"

static GyroData g_data;
static int      g_has_new;

void gyro_init(void)
{
    g_data.wz  = 0.0f;
    g_data.yaw = 0.0f;
    g_has_new  = 0;
}

void gyro_feed_byte(uint8_t ucData)
{
    static uint8_t buf[5];
    static uint8_t cnt = 0;

    buf[cnt++] = ucData;

    /* 帧头必须为 0x5A */
    if (buf[0] != 0x5A)
    {
        cnt = 0;
        return;
    }

    /* 不满 5 字节继续等 */
    if (cnt < 5) return;

    /* ---- 校验 ---- */
    uint8_t sum = buf[0] + buf[1] + buf[2] + buf[3];
    if (sum != buf[4])
    {
        cnt = 0;
        return;
    }

    short raw = (short)((buf[3] << 8) | buf[2]);

    if (buf[1] == 0xAA)   /* 角速度帧 */
    {
        g_data.wz  = (float)raw / 32768.0f * 2000.0f;
        g_has_new  = 1;
    }
    else if (buf[1] == 0xBB)   /* 角度帧 */
    {
        g_data.yaw = (float)raw / 32768.0f * 180.0f;
        g_has_new  = 1;
    }
    /* 其他 type 忽略 */

    cnt = 0;
}

int gyro_has_new_data(void)
{
    return g_has_new;
}

GyroData gyro_get_data(void)
{
    g_has_new = 0;
    return g_data;
}
