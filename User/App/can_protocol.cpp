/**
 * @file can_protocol.cpp
 * @brief CAN 协议 — 指令解析 + 定时上报状态
 *
 * 主机 → MSPM0: CAN ID 0x100, Data[0]=cmd, Data[1-2]=param1, Data[3-4]=param2
 * MSPM0 → 主机: CAN ID 0x200, Data[0]=status, Data[1-2]=angle×100, Data[3-4]=speed×10
 */

#include "can_protocol.hpp"
#include "bsp_can.h"
#include "motor_control.hpp"

static uint32_t g_tick = 0;

void can_proto_init(void)
{
    bsp_can_init();
}

void can_proto_tick(void)
{
    g_tick++;

    /* ---- 处理接收 ---- */
    uint32_t rx_id;
    uint8_t  rx_data[8];
    uint8_t  rx_len;

    if (bsp_can_recv(&rx_id, rx_data, &rx_len))
    {
        if (rx_id == CAN_ID_CMD && rx_len >= 3)
        {
            uint8_t  cmd    = rx_data[0];
            int16_t  param1 = (int16_t)((rx_data[1] << 8) | rx_data[2]);
            int16_t  param2 = (int16_t)((rx_data[3] << 8) | rx_data[4]);

            switch (cmd)
            {
                case CMD_SET_ANGLE: {
                    float angle   = (float)param1 / 100.0f;      // 角度 °
                    float max_rpm = (float)param2;               // 限速 RPM
                    motor_move_to_ex(angle, max_rpm);
                    break;
                }
                case CMD_STOP:
                    motor_stop();
                    break;
                case CMD_OPEN_LOOP: {
                    float rpm = (float)param1 / 10.0f;
                    motor_set_speed(rpm);
                    break;
                }
                default:
                    break;
            }
        }
    }

    /* ---- 每 50ms 上报一次状态 ---- */
    if ((g_tick % 50) == 0)
    {
        uint8_t tx[8];

        int stat = STAT_IDLE;
        if (motor_move_done())
            stat = STAT_DONE;
        else if (motor_target_speed() != 0.0f || motor_is_moving())
            stat = STAT_MOVING;

        int16_t angle_x100 = (int16_t)(motor_angle() * 100.0f);
        int16_t speed_x10  = (int16_t)(motor_speed() * 10.0f);

        tx[0] = (uint8_t)stat;
        tx[1] = (uint8_t)(angle_x100 >> 8);
        tx[2] = (uint8_t)(angle_x100);
        tx[3] = (uint8_t)(speed_x10 >> 8);
        tx[4] = (uint8_t)(speed_x10);
        tx[5] = 0;  // 错误码保留

        bsp_can_send(CAN_ID_STAT, tx, 6);
    }
}
