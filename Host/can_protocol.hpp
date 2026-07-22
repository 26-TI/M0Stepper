/**
 * @file can_protocol.hpp
 * @brief 主机端 CAN 电机控制类 — 命令发送 + 状态缓存
 *
 * 用法:
 *   CanProtocol can;
 *   can.init();
 *   can.tick();                      // 每 1ms
 *   can.setSpeed(1, 60);             // 电机1 正转 60 RPM
 *   float ang = can.getAngle(1);     // 读电机1 角度
 */

#ifndef __CAN_PROTOCOL_HPP__
#define __CAN_PROTOCOL_HPP__

#include <stdint.h>
#include <stdbool.h>
#include "m0stepper_host.h"

#define CAN_MAX_MOTORS 8

class CanProtocol
{
public:
    CanProtocol() = default;
    void init();        ///< 初始化 CAN + 绑定发送函数
    void tick();        ///< 每 1ms: 收状态帧 → 更新缓存

    /* ---- 电机控制命令 ---- */
    void setSpeed(uint8_t id, float rpm);
    void moveTo(uint8_t id, float angle_deg, float max_rpm = 45.0f);
    void moveAbs(uint8_t id, float angle_deg, int turns, float max_rpm);
    void timedMove(uint8_t id, float angle_deg, float duration_s);
    void stop(uint8_t id);
    void enable(uint8_t id, bool on);
    void query(uint8_t id, uint8_t sub = 0);

    /* ---- 状态读取 ---- */
    bool  hasStatus(uint8_t id) const;
    float getAngle(uint8_t id) const;
    float getSpeed(uint8_t id) const;
    int   getTurns(uint8_t id) const;
    int   getState(uint8_t id) const;
    bool  isDone(uint8_t id) const;

private:
    M0Stepper::Status status_[CAN_MAX_MOTORS];  ///< 状态缓存
    bool              valid_[CAN_MAX_MOTORS] = {};

    void sendCmd_(uint8_t motorId, uint8_t cmd, int16_t p1, int16_t p2, int16_t p3);
};

/* 全局实例 */
extern CanProtocol can_proto;

#endif
