/**
 * @file m0stepper_host.h
 * @brief 主控端 CAN 电机控制库（纯 C，单头文件）
 *
 * 用法:
 *   1. 在你的主控工程里 #include "m0stepper_host.h"
 *   2. 初始化: m0stepper_bind(CAN_Send);   // 绑定你的 CAN 发送函数
 *   3. 调用:
 *        m0st_set_speed(1, 60);            // 电机1 正转 60 RPM
 *        m0st_move_to(1, 90, 45);          // 电机1 转到 90° 限速45
 *        m0st_enable(1, true);             // 电机1 使能
 *        m0st_stop(1);                     // 电机1 停止
 */

#ifndef __M0STEPPER_HOST_H__
#define __M0STEPPER_HOST_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  协议常量
 * ================================================================ */

enum {
    M0ST_CMD_SPEED    = 0x01,
    M0ST_CMD_POSITION = 0x02,
    M0ST_CMD_STOP     = 0x03,
    M0ST_CMD_ENABLE   = 0x04,
    M0ST_CMD_QUERY    = 0x05,
};

enum {
    M0ST_QUERY_ALL    = 0x00,
    M0ST_QUERY_ANGLE  = 0x01,
    M0ST_QUERY_SPEED  = 0x02,
    M0ST_QUERY_TURNS  = 0x03,
};

enum {
    M0ST_STAT_IDLE   = 0,
    M0ST_STAT_MOVING = 1,
    M0ST_STAT_DONE   = 2,
    M0ST_STAT_ERROR  = 3,
};

/* ================================================================
 *  发送函数指针（主控初始化时绑定一次）
 * ================================================================ */

/** CAN 发送函数类型: send(id, data, len) */
typedef void (*m0stepper_send_fn)(uint32_t id, const uint8_t *data, uint8_t len);

static m0stepper_send_fn g_m0st_send = NULL;

/** 绑定 CAN 发送函数。传入你的 CAN_Send（或封装后的函数） */
static inline void m0stepper_bind(m0stepper_send_fn fn)
{
    g_m0st_send = fn;
}

/* ================================================================
 *  CAN 帧构建
 * ================================================================ */

/** XOR(B0~B6) */
static inline uint8_t m0st_checksum(const uint8_t d[8])
{
    return d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6];
}

/** 打包 int16 大端到 buf[pos] */
static inline void m0st_pack_i16(uint8_t *buf, int pos, int16_t v)
{
    buf[pos]   = (uint8_t)(v >> 8);
    buf[pos+1] = (uint8_t)v;
}

/** 构建 8 字节命令帧 */
static inline void m0st_build(uint8_t d[8],
                              uint8_t cmd, int16_t p1, int16_t p2, int16_t p3)
{
    d[0] = cmd;
    m0st_pack_i16(d, 1, p1);
    m0st_pack_i16(d, 3, p2);
    m0st_pack_i16(d, 5, p3);
    d[7] = m0st_checksum(d);
}

/* ================================================================
 *  API: 电机控制命令
 *
 *  每个函数两个参数 (motor_id, ...)
 *    motor_id: 电机设备号 1~N
 *    CAN ID 自动计算: 命令=0x100+N, 状态=0x200+N
 * ================================================================ */

/* ---- 速度控制 ---- */

/** 速度控制（开环）
 *  @param motor_id  电机号 (1~N)
 *  @param rpm       目标转速 RPM，正=正转，负=反转
 */
static inline void m0st_set_speed(uint8_t motor_id, float rpm)
{
    uint8_t d[8];
    m0st_build(d, M0ST_CMD_SPEED, (int16_t)(rpm * 10.0f), 0, 0);
    g_m0st_send(0x100 + motor_id, d, 8);
}

/* ---- 位置控制 ---- */

/** 多圈位置控制（闭环 P）
 *  @param motor_id  电机号
 *  @param angle_deg 目标角度 0°~360°
 *  @param turns     目标圈数（可正可负）
 *  @param max_rpm   最大转速 RPM（0=默认45）
 */
static inline void m0st_move_to_multi(uint8_t motor_id,
                                      float angle_deg, int16_t turns, float max_rpm)
{
    uint8_t d[8];
    m0st_build(d, M0ST_CMD_POSITION,
               (int16_t)(angle_deg * 100.0f), turns, (int16_t)max_rpm);
    g_m0st_send(0x100 + motor_id, d, 8);
}

/** 单圈位置控制（闭环 P）
 *  @param motor_id  电机号
 *  @param angle_deg 目标角度 0°~360°
 *  @param max_rpm   最大转速 RPM（0=默认45）
 */
static inline void m0st_move_to(uint8_t motor_id,
                                float angle_deg, float max_rpm)
{
    m0st_move_to_multi(motor_id, angle_deg, 0, max_rpm);
}

/* ---- 停止 ---- */

static inline void m0st_stop(uint8_t motor_id)
{
    uint8_t d[8];
    m0st_build(d, M0ST_CMD_STOP, 0, 0, 0);
    g_m0st_send(0x100 + motor_id, d, 8);
}

/* ---- 使能/失能 ---- */

/** 使能/失能驱动器
 *  @param on  true=使能(PA25低), false=失能(PA25高)
 */
static inline void m0st_enable(uint8_t motor_id, bool on)
{
    uint8_t d[8];
    m0st_build(d, M0ST_CMD_ENABLE, on ? 1 : 0, 0, 0);
    g_m0st_send(0x100 + motor_id, d, 8);
}

/* ---- 查询 ---- */

/** 查询电机状态（发完立刻收一帧回复）
 *  @param motor_id  电机号
 *  @param sub       查询码: 0=全状态, 1=仅角度, 2=仅速度, 3=仅圈数
 */
static inline void m0st_query(uint8_t motor_id, uint8_t sub)
{
    uint8_t d[8];
    m0st_build(d, M0ST_CMD_QUERY, sub, 0, 0);
    g_m0st_send(0x100 + motor_id, d, 8);
}

/* ================================================================
 *  API: 状态帧解析
 *
 *  主控收到 CAN ID 0x200+N 的帧后调 m0st_parse_status() 解析
 * ================================================================ */

/** 电机状态数据 */
typedef struct {
    uint8_t state;      /* 0=空闲 1=运动中 2=到位 3=故障 */
    float   angle_deg;  /* 当前角度 ° */
    float   rpm;        /* 当前转速 RPM */
    int16_t turns;      /* 累计圈数 */
    bool    ck_ok;      /* 校验和正确？ */
} M0Stepper_Status;

/** 解析状态帧
 *  @param data  收到的 8 字节 CAN 数据
 *  @param st    输出状态结构体
 */
static inline void m0st_parse_status(const uint8_t d[8], M0Stepper_Status *st)
{
    st->ck_ok = (d[7] == m0st_checksum(d));
    st->state = d[0];
    st->angle_deg = (float)((int16_t)((d[1]<<8)|d[2])) / 100.0f;
    st->rpm       = (float)((int16_t)((d[3]<<8)|d[4])) / 10.0f;
    st->turns     = (int16_t)((d[5]<<8)|d[6]);
}

/** 快捷：判断电机是否到位 */
static inline bool m0st_is_done(const M0Stepper_Status *st)
{
    return st->ck_ok && st->state == M0ST_STAT_DONE;
}

/** 快捷：判断电机是否运动中 */
static inline bool m0st_is_moving(const M0Stepper_Status *st)
{
    return st->ck_ok && st->state == M0ST_STAT_MOVING;
}

#ifdef __cplusplus
}
#endif

#endif /* __M0STEPPER_HOST_H__ */
