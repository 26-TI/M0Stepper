/**
 * @file imu601.cpp
 * @brief 汇电籽-601 协议解析实现（Device 层，纯 C）
 *
 * === 状态机 ===
 *
 *   等 0xAA → 等 0x55 → 收满 12 字节 → 校验 → 解析 Payload → 回到等 0xAA
 *
 *   用 last 变量记住上一字节，检测连续的 AA 55 帧头。
 *   数据流中如果出现 AA 55 模式的 payload 内容，会被误当帧头——
 *   但实际数据（yaw/pitch/roll 的 int16 范围）很少出现这种组合，
 *   即使误触发也会被校验和过滤掉，不影响正确数据。
 *
 * === 校验（和参考例程 car_06_IMU601 一致）===
 *
 *   cksum = buf[2] + buf[3] + ... + buf[10]
 *   不包含帧头 AA 55 和校验字节本身。
 *
 * === 解析 ===
 *
 *   Payload 6 字节，小端：
 *   [0][1] Yaw   uint16 → ÷100.0
 *   [2][3] Pitch int16  → ÷100.0
 *   [4][5] Roll  int16  → ÷100.0
 */

#include "imu601.h"

static IMU601_Data g_data;
static int         g_has_new;

/* 解析 payload 6 字节 → Yaw/Pitch/Roll */
static void parse_payload(const uint8_t *p)
{
    uint16_t yaw_raw   = ((uint16_t)p[1] << 8) | p[0];
    int16_t  pitch_raw = (int16_t)(((uint16_t)p[3] << 8) | p[2]);
    int16_t  roll_raw  = (int16_t)(((uint16_t)p[5] << 8) | p[4]);

    g_data.yaw   = (float)yaw_raw   / 100.0f;
    g_data.pitch = (float)pitch_raw / 100.0f;
    g_data.roll  = (float)roll_raw  / 100.0f;
    g_has_new = 1;
}

void imu601_init(void)
{
    g_data.yaw   = 0.0f;
    g_data.pitch = 0.0f;
    g_data.roll  = 0.0f;
    g_has_new    = 0;
}

void imu601_feed_byte(uint8_t byte)
{
    static uint8_t buf[12];   /* 帧缓冲区 */
    static uint8_t idx  = 0;  /* 当前写入位置 */
    static uint8_t last = 0;  /* 上一字节（用于检测 AA 55） */
    static uint8_t sync = 0;  /* 0=正在找帧头, 1=已同步正在收帧体 */

    buf[idx++] = byte;

    if (!sync)
    {
        /* 扫描连续的 AA 55 */
        if (last == 0xAA && byte == 0x55)
        {
            /* 确认帧头，重置缓冲区从 idx=2 开始收 */
            buf[0] = 0xAA;
            buf[1] = 0x55;
            idx    = 2;
            sync   = 1;
        }
        last = byte;
        return;
    }

    /* sync=1：正在收帧体，等收满 12 字节 */
    if (idx >= 12)
    {
        /* 校验：sum(DevID + Cmd + Len + Payload) */
        uint8_t cksum = 0;
        for (int i = 2; i < 11; i++)
            cksum += buf[i];

        if (cksum == buf[11])
        {
            /* 确认是指令 0x01（姿态数据），长度 0x06 */
            if (buf[3] == 0x01 && buf[4] == 0x06)
                parse_payload(&buf[5]);
        }
        /* 校验失败或指令不匹配：静默丢弃，重新找帧头 */

        idx  = 0;
        sync = 0;
    }
}

int imu601_has_new_data(void)
{
    return g_has_new;
}

IMU601_Data imu601_get_data(void)
{
    g_has_new = 0;   /* 读后清标记，避免重复返回旧数据 */
    return g_data;
}
