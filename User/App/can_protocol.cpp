/**
 * @file can_protocol.cpp
 * @brief CAN 协议 — 8 字节帧 + 多圈 + 校验
 *
 * 命令 0x10N: [0]cmd [1-2]p1 i16 [3-4]p2 i16 [5-6]p3 i16 [7]CK
 * 状态 0x20N: [0]stat [1-2]ang×100 i16 [3-4]rpm×10 i16 [5-6]turns i16 [7]CK
 */

#define CAN_USE_IRQ 1

#include "can_protocol.hpp"
#include "bsp_can.hpp"
#include "bsp_uart.h"
#include "motor_control.hpp"
#include "stepper_motor.h"

static uint32_t g_tick;
static uint32_t g_rx_cnt = 0;   ///< 收帧计数 (调试用)

/* ---- 组装一帧状态并发送 ---- */
static void send_status_frame(int16_t ang_x100, int16_t spd_x10, int16_t turns,
                               uint8_t stat)
{
    uint8_t tx[8];
    tx[0] = stat;
    tx[1] = ang_x100 >> 8;   tx[2] = ang_x100;
    tx[3] = spd_x10  >> 8;   tx[4] = spd_x10;
    tx[5] = turns     >> 8;   tx[6] = turns;
    tx[7] = can_checksum(tx, 7);
    CAN_Send(CAN_ID_STAT, tx, 8);
}

void can_proto_init(void)
{
    CAN_Init(CAN_MODE_NORMAL);
#if CAN_USE_IRQ
    CAN_EnableIrq();
#endif
}

void can_proto_tick(void)
{
    g_tick++;

#if !CAN_USE_IRQ
    CAN_RecvPoll();
#endif

    /* ---- 收指令 ---- */
    uint32_t id; uint8_t d[8], len;
    while (CAN_RecvRead(&id, d, &len))
    {
        g_rx_cnt++;
        bsp_uart_printf("CAN RX[%u] ID=0x%03X D=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                        (unsigned)g_rx_cnt, (unsigned)id,
                        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
        if (id == CAN_ID_CMD && len >= 8 && d[7] == can_checksum(d, 7))
        {
            int16_t p1 = (d[1]<<8)|d[2], p2 = (d[3]<<8)|d[4];

            switch (d[0])
            {
                case CMD_SPEED:
                    motor_set_speed(p1 / 10.0f);
                    break;

                case CMD_POSITION:
                    if (p2 != 0)
                        motor_move_abs(p1 / 100.0f, p2, (int16_t)((d[5]<<8)|d[6]));
                    else
                        motor_move_to_ex(p1 / 100.0f, (float)(int16_t)((d[5]<<8)|d[6]));
                    break;

                case CMD_TIMED:
                    motor_timed_move(p1 / 100.0f, (float)p2 / 1000.0f);
                    break;

                case CMD_STOP:
                    motor_stop();
                    break;

                case CMD_ENABLE:
                    bsp_uart_printf("CAN CMD_ENABLE p1=%d\r\n", p1);
                    stepper_enable(p1 != 0);
                    break;

                case CMD_QUERY:
                {
                    int16_t a = (int16_t)(motor_angle() * 100.0f);
                    int16_t s = (int16_t)(motor_speed() * 10.0f);
                    int16_t t = (int16_t)motor_get_turns();
                    uint8_t st = motor_move_done() ? STAT_DONE :
                                 (motor_target_speed()!=0||motor_is_moving()) ? STAT_MOVING : STAT_IDLE;

                    switch (p1)
                    {
                        case 0x01: a = (int16_t)(motor_angle() * 100.0f); s = 0; t = 0; break; // 仅角度
                        case 0x02: a = 0; s = (int16_t)(motor_speed() * 10.0f); t = 0; break;  // 仅速度
                        case 0x03: a = 0; s = 0; t = (int16_t)motor_get_turns(); break;         // 仅圈数
                        default: break;  // 0x00=全状态
                    }
                    send_status_frame(a, s, t, st);
                    break;
                }
            }
        }
    }

    /* ---- 50ms 定时上报 ---- */
    if ((g_tick % 50) == 0)
    {
        int16_t a = (int16_t)(motor_angle() * 100.0f);
        int16_t s = (int16_t)(motor_speed() * 10.0f);
        int16_t t = (int16_t)motor_get_turns();
        uint8_t st = motor_move_done() ? STAT_DONE :
                     (motor_target_speed()!=0||motor_is_moving()) ? STAT_MOVING : STAT_IDLE;
        send_status_frame(a, s, t, st);
    }
}
