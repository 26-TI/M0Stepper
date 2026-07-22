/**
 * @file m0stepper_host.h
 * @brief 主控端 CAN 电机控制库 — C++ 类封装
 *
 * 用法:
 *   #include "m0stepper_host.h"
 *
 *   M0Stepper motor(1);                   // 控电机 1
 *   motor.bind(CAN_Send);                 // 绑定 CAN 发送函数
 *
 *   motor.enable();                       // 使能
 *   motor.setSpeed(60);                   // 正转 60 RPM
 *   motor.moveTo(90);                     // 就近到 90°
 *   motor.moveAbs(90, 3, 60);             // 绝对: 3圈90° 限速60
 *   motor.timedMove(90, 2.0f);            // 2秒内转到90°
 *   motor.stop();                         // 停止
 *
 *   // 收到状态帧时解析:
 *   if (id == 0x201) {
 *       M0Stepper::Status st;
 *       M0Stepper::parseStatus(data, &st);
 *       if (st.done()) { ... }
 *   }
 */

#ifndef __M0STEPPER_HOST_H__
#define __M0STEPPER_HOST_H__

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus

/* ================================================================
 *  协议常量
 * ================================================================ */

enum : uint8_t {
    CMD_SPEED    = 0x01,
    CMD_POSITION = 0x02,
    CMD_STOP     = 0x03,
    CMD_ENABLE   = 0x04,
    CMD_QUERY    = 0x05,
    CMD_TIMED    = 0x06,
};

enum : uint8_t {
    QUERY_ALL   = 0x00,
    QUERY_ANGLE = 0x01,
    QUERY_SPEED = 0x02,
    QUERY_TURNS = 0x03,
};

/* ================================================================
 *  CAN 帧编解码
 * ================================================================ */

static inline uint8_t canChecksum(const uint8_t d[8])
{
    return d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6];
}

static inline void packI16(uint8_t *buf, int pos, int16_t v)
{
    buf[pos]   = (uint8_t)(v >> 8);
    buf[pos+1] = (uint8_t)v;
}

static inline void buildFrame(uint8_t d[8],
                              uint8_t cmd, int16_t p1, int16_t p2, int16_t p3)
{
    d[0] = cmd;
    packI16(d, 1, p1);
    packI16(d, 3, p2);
    packI16(d, 5, p3);
    d[7] = canChecksum(d);
}

/* ================================================================
 *  M0Stepper 类
 * ================================================================ */

class M0Stepper
{
public:
    /** 电机状态 */
    struct Status {
        uint8_t state = 0;       // 0=空闲 1=运动 2=到位 3=故障
        float   angle = 0;       // 当前角度 °
        float   rpm   = 0;       // 当前转速 RPM
        int16_t turns = 0;       // 累计圈数
        bool    ckOk  = false;   // 校验和正确?

        bool isDone()   const { return ckOk && state == 2; }
        bool isMoving() const { return ckOk && state == 1; }
    };

    /** CAN 发送函数类型 */
    typedef void (*SendFn)(uint32_t id, const uint8_t *data, uint8_t len);

    M0Stepper(uint8_t motorId = 1) : motorId_(motorId) {}

    /** 绑定 CAN 发送函数（初始化时调用一次） */
    void bind(SendFn fn) { send_ = fn; }

    /* ---- 控制命令 ---- */

    void setSpeed(float rpm) {
        uint8_t d[8];
        buildFrame(d, CMD_SPEED, (int16_t)(rpm * 10.0f), 0, 0);
        send_(cmdId(), d, 8);
    }

    void moveTo(float angle_deg, float max_rpm = 0) {
        uint8_t d[8];
        buildFrame(d, CMD_POSITION, (int16_t)(angle_deg * 100.0f), 0, (int16_t)max_rpm);
        send_(cmdId(), d, 8);
    }

    void moveAbs(float angle_deg, int16_t turns, float max_rpm = 45) {
        uint8_t d[8];
        buildFrame(d, CMD_POSITION, (int16_t)(angle_deg * 100.0f), turns, (int16_t)max_rpm);
        send_(cmdId(), d, 8);
    }

    void timedMove(float angle_deg, float duration_s) {
        uint8_t d[8];
        buildFrame(d, CMD_TIMED, (int16_t)(angle_deg * 100.0f), (int16_t)(duration_s * 1000.0f), 0);
        send_(cmdId(), d, 8);
    }

    void stop() {
        uint8_t d[8];
        buildFrame(d, CMD_STOP, 0, 0, 0);
        send_(cmdId(), d, 8);
    }

    void enable(bool on = true) {
        uint8_t d[8];
        buildFrame(d, CMD_ENABLE, on ? 1 : 0, 0, 0);
        send_(cmdId(), d, 8);
    }

    void disable() { enable(false); }

    void query(uint8_t sub = QUERY_ALL) {
        uint8_t d[8];
        buildFrame(d, CMD_QUERY, sub, 0, 0);
        send_(cmdId(), d, 8);
    }

    /* ---- 状态解析 ---- */

    static void parseStatus(const uint8_t d[8], Status *st) {
        st->ckOk  = (d[7] == canChecksum(d));
        st->state = d[0];
        st->angle = (float)((int16_t)((d[1]<<8)|d[2])) / 100.0f;
        st->rpm   = (float)((int16_t)((d[3]<<8)|d[4])) / 10.0f;
        st->turns = (int16_t)((d[5]<<8)|d[6]);
    }

    uint32_t cmdId()  const { return 0x100 + motorId_; }
    uint32_t statId() const { return 0x200 + motorId_; }

private:
    uint8_t motorId_;
    SendFn  send_ = nullptr;
};

#endif /* __cplusplus */
#endif /* __M0STEPPER_HOST_H__ */
