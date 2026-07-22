/**
 * @file can_protocol.cpp
 * @brief 主机端 CAN 电机控制类实现 — 用 M0Stepper::buildFrame + sendFn
 */

#include "can_protocol.hpp"
#include "bsp_can.h"
#include <string.h>

/* ---- 全局实例 ---- */
CanProtocol can_proto;

/* ================================================================
 *  生命周期
 * ================================================================ */

void CanProtocol::init()
{
    bsp_can.init();
    bsp_can.enable_irq();
    memset(valid_, 0, sizeof(valid_));
}

void CanProtocol::tick()
{
    uint32_t rx_id;
    uint8_t  d[8], len;

    while (bsp_can.recv_read(&rx_id, d, &len))
    {
        if (rx_id >= 0x201 && rx_id <= 0x200 + CAN_MAX_MOTORS && len >= 8)
        {
            uint8_t id = (uint8_t)(rx_id - 0x200);
            if (id >= 1 && id <= CAN_MAX_MOTORS)
            {
                M0Stepper::parseStatus(d, &status_[id - 1]);
                valid_[id - 1] = status_[id - 1].ckOk;
            }
        }
    }
}

/* ================================================================
 *  发送辅助
 * ================================================================ */

void CanProtocol::sendCmd_(uint8_t motorId, uint8_t cmd, int16_t p1, int16_t p2, int16_t p3)
{
    uint8_t d[8];
    buildFrame(d, cmd, p1, p2, p3);
    bsp_can.send(0x100 + motorId, d, 8);
}

/* ================================================================
 *  电机控制命令
 * ================================================================ */

void CanProtocol::setSpeed(uint8_t id, float rpm)
    { sendCmd_(id, CMD_SPEED, (int16_t)(rpm * 10.0f), 0, 0); }

void CanProtocol::moveTo(uint8_t id, float ang, float max_rpm)
    { sendCmd_(id, CMD_POSITION, (int16_t)(ang * 100.0f), 0, (int16_t)max_rpm); }

void CanProtocol::moveAbs(uint8_t id, float ang, int turns, float max_rpm)
    { sendCmd_(id, CMD_POSITION, (int16_t)(ang * 100.0f), turns, (int16_t)max_rpm); }

void CanProtocol::timedMove(uint8_t id, float ang, float dur)
    { sendCmd_(id, CMD_TIMED, (int16_t)(ang * 100.0f), (int16_t)(dur * 1000.0f), 0); }

void CanProtocol::stop(uint8_t id)   { sendCmd_(id, CMD_STOP,   0, 0, 0); }
void CanProtocol::enable(uint8_t id, bool on) { sendCmd_(id, CMD_ENABLE, on ? 1 : 0, 0, 0); }
void CanProtocol::query(uint8_t id, uint8_t sub) { sendCmd_(id, CMD_QUERY, sub, 0, 0); }

/* ================================================================
 *  状态读取
 * ================================================================ */

bool  CanProtocol::hasStatus(uint8_t id) const  { return id >= 1 && id <= CAN_MAX_MOTORS && valid_[id - 1]; }
float CanProtocol::getAngle(uint8_t id) const   { return hasStatus(id) ? status_[id - 1].angle : 0.0f; }
float CanProtocol::getSpeed(uint8_t id) const   { return hasStatus(id) ? status_[id - 1].rpm : 0.0f; }
int   CanProtocol::getTurns(uint8_t id) const   { return hasStatus(id) ? status_[id - 1].turns : 0; }
int   CanProtocol::getState(uint8_t id) const   { return hasStatus(id) ? status_[id - 1].state : -1; }
bool  CanProtocol::isDone(uint8_t id) const     { return hasStatus(id) && status_[id - 1].isDone(); }
