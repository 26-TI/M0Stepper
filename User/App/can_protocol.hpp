/**
 * @file can_protocol.hpp
 * @brief CAN 通信协议类 — 指令解析 + 状态上报
 */

#ifndef __CAN_PROTOCOL_HPP__
#define __CAN_PROTOCOL_HPP__

#include <stdint.h>

#ifndef MOTOR_ID
  #define MOTOR_ID   1
#endif

#define CAN_ID_CMD   (0x100 + MOTOR_ID)
#define CAN_ID_STAT  (0x200 + MOTOR_ID)

/* ---- 命令码 ---- */
enum : uint8_t {
    CMD_SPEED    = 0x01,   ///< 纯速度: p1=RPM×10
    CMD_POSITION = 0x02,   ///< 位置: p1=角度×100, p2=圈数, p3=最大RPM
    CMD_STOP     = 0x03,   ///< 立即停止
    CMD_ENABLE   = 0x04,   ///< 使能/失能: p1=1使能 0失能
    CMD_QUERY    = 0x05,   ///< 查询: p1=0全 1角 2速 3圈
    CMD_TIMED    = 0x06,   ///< 定时转动: p1=角度×100, p2=时长ms
};

/* ---- 状态码 ---- */
enum : uint8_t {
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

class MotorControl;  // forward

class CanProtocol
{
public:
    CanProtocol(MotorControl &motor, uint8_t id = MOTOR_ID);

    void init();       ///< 初始化 CAN + 滤波器 + 中断
    void tick();       ///< 每 1ms 调用

private:
    MotorControl &motor_;
    uint8_t       motorId_;
    uint32_t      tick_  = 0;
    uint32_t      rxCnt_ = 0;   // debug

    void sendStatus(int16_t ang_x100, int16_t spd_x10, int16_t turns,
                    uint8_t stat);
};

#endif /* __CAN_PROTOCOL_HPP__ */
