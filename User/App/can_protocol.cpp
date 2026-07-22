/**
 * @file can_protocol.cpp
 * @brief CAN 协议类实现 — 指令解析 + 状态上报
 */

#define CAN_USE_IRQ 1

#include "can_protocol.hpp"
#include "motor_control.hpp"
#include "bsp_can.hpp"
#include "stepper_motor.h"

CanProtocol::CanProtocol(MotorControl &motor, uint8_t id)
    : motor_(motor), motorId_(id)
{}

void CanProtocol::init()
{
    bsp_can.init(BspCan::NORMAL);
#if CAN_USE_IRQ
    bsp_can.enableIrq();
#endif
}

void CanProtocol::tick()
{
    tick_++;

#if !CAN_USE_IRQ
    bsp_can.recvPoll();
#endif

    /* ---- 收指令 ---- */
    uint32_t id; uint8_t d[8], len;
    while (bsp_can.recvRead(&id, d, &len))
    {
        rxCnt_++;
        // bsp_uart_printf CAN RX 太慢(5ms)，锁内禁用，只在定时调试时打开

        if (id == CAN_ID_CMD && len >= 8 && d[7] == can_checksum(d, 7))
        {
            int16_t p1 = (d[1]<<8)|d[2], p2 = (d[3]<<8)|d[4];

            switch (d[0])
            {
                case CMD_SPEED:
                    motor_.setSpeed(p1 / 10.0f);
                    break;

                case CMD_POSITION:
                    if (p2 != 0)
                        motor_.moveAbs(p1 / 100.0f, p2, (int16_t)((d[5]<<8)|d[6]));
                    else
                        motor_.moveTo(p1 / 100.0f, (float)(int16_t)((d[5]<<8)|d[6]));
                    break;

                case CMD_TIMED:
                    motor_.timedMove(p1 / 100.0f, (float)p2 / 1000.0f);
                    break;

                case CMD_STOP:
                    motor_.stop();
                    break;

                case CMD_ENABLE:
                    // bsp_uart_printf CAN CMD enabled (takes 5ms in mutex)
                    stepper_enable(p1 != 0);
                    break;

                case CMD_QUERY:
                {
                    int16_t a = (int16_t)(motor_.angle() * 100.0f);
                    int16_t s = (int16_t)(motor_.speed() * 10.0f);
                    int16_t t = (int16_t)motor_.turns();
                    uint8_t st = motor_.isDone()     ? STAT_DONE :
                                 motor_.isMoving() ? STAT_MOVING : STAT_IDLE;

                    switch (p1)
                    {
                        case 0x01: a = (int16_t)(motor_.angle() * 100.0f); s = 0; t = 0; break;
                        case 0x02: a = 0; s = (int16_t)(motor_.speed() * 10.0f); t = 0; break;
                        case 0x03: a = 0; s = 0; t = (int16_t)motor_.turns();    break;
                        default: break;
                    }
                    sendStatus(a, s, t, st);
                    break;
                }
            }
        }
    }

    /* ---- 50ms 定时上报 ---- */
    if ((tick_ % 50) == 0)
    {
        int16_t a = (int16_t)(motor_.angle() * 100.0f);
        int16_t s = (int16_t)(motor_.speed() * 10.0f);
        int16_t t = (int16_t)motor_.turns();
        uint8_t st = motor_.isDone()     ? STAT_DONE :
                     motor_.isMoving() ? STAT_MOVING : STAT_IDLE;
        sendStatus(a, s, t, st);
    }
}

void CanProtocol::sendStatus(int16_t ang_x100, int16_t spd_x10, int16_t turns,
                              uint8_t stat)
{
    uint8_t tx[8];
    tx[0] = stat;
    tx[1] = ang_x100 >> 8;   tx[2] = ang_x100;
    tx[3] = spd_x10  >> 8;   tx[4] = spd_x10;
    tx[5] = turns     >> 8;   tx[6] = turns;
    tx[7] = can_checksum(tx, 7);
    bsp_can.send(CAN_ID_STAT, tx, 8);
}
