/**
 * @file can_protocol.hpp
 * @brief CAN 通信协议 — 指令解析 + 状态上报（多设备支持）
 *
 * 每个电机分配独立 CAN ID：
 *   电机 N: 命令 ID = 0x100 + N, 状态 ID = 0x200 + N
 */

#ifndef __CAN_PROTOCOL_HPP__
#define __CAN_PROTOCOL_HPP__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOTOR_ID
  #define MOTOR_ID   1              ///< 本机设备号（编译时定义）
#endif

#define CAN_ID_CMD   (0x100 + MOTOR_ID)  ///< 主机→MSPM0 指令
#define CAN_ID_STAT  (0x200 + MOTOR_ID)  ///< MSPM0→主机 状态

/* ================================================================
 *  命令帧 0x10N (主机→MSPM0, 8 字节)
 *    [0]命令 [1-2]p1 i16 BE [3-4]p2 i16 BE [5-6]p3 i16 BE [7]XOR(B0~B6)
 * ================================================================ */

enum {
    CMD_SPEED    = 0x01,   ///< 纯速度: p1=RPM×10
    CMD_POSITION = 0x02,   ///< 位置控制: p1=角度×100, p2=圈数 int16, p3=最大RPM(0=默认45)
    CMD_STOP     = 0x03,   ///< 立即停止
    CMD_ENABLE   = 0x04,   ///< 使能/失能: p1=1使能 0失能
    CMD_QUERY    = 0x05,   ///< 查询: p1=0全状态 1角度 2速度 3圈数
};

/* ================================================================
 *  状态帧 0x20N (MSPM0→主机, 8 字节)
 *    [0]状态 [1-2]角度×100 i16 [3-4]转速×10 i16
 *    [5-6]累计圈数 i16 BE [7]XOR(B0~B6)
 * ================================================================ */

enum {
    STAT_IDLE   = 0,
    STAT_MOVING = 1,
    STAT_DONE   = 2,
    STAT_ERROR  = 3,
};

static inline uint8_t can_checksum(const uint8_t *d, int n)
{
    uint8_t c = 0;
    for (int i = 0; i < n; i++) c ^= d[i];
    return c;
}

void can_proto_init(void);          ///< 初始化 + 配置 CAN 滤波器
void can_proto_tick(void);          ///< 每 1ms 调用

#ifdef __cplusplus
}
#endif

#endif
