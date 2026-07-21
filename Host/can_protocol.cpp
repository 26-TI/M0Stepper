/**
 * @file can_protocol.cpp
 * @brief 主控端 CAN 电机控制库实现
 *
 * 发送: m0stepper_host.h 构建命令 → bsp_can_send()
 * 接收: bsp_can_recv_read() → 匹配 0x200+N → m0st_parse_status() → 缓存
 */

#include "can_protocol.hpp"
#include "bsp_can.h"
#include <string.h>

#define MAX_MOTORS 8

static M0Stepper_Status g_st[MAX_MOTORS];
static bool             g_st_ok[MAX_MOTORS];

/* ---- 发送包装（m0stepper_send_fn）---- */
static void send_fn(uint32_t id, const uint8_t *data, uint8_t len)
{
    bsp_can_send(id, (uint8_t *)data, len);
}

/* ================================================================
 *  生命周期
 * ================================================================ */

void can_motor_init(void)
{
    bsp_can_init();
    bsp_can_enable_irq();
    m0stepper_bind(send_fn);
    memset(g_st_ok, 0, sizeof(g_st_ok));
}

void can_motor_tick(void)
{
    uint32_t rx_id;
    uint8_t  d[8], len;

    while (bsp_can_recv_read(&rx_id, d, &len))
    {
        if (rx_id >= 0x201 && rx_id <= 0x200 + MAX_MOTORS && len >= 8)
        {
            uint8_t id = (uint8_t)(rx_id - 0x200);
            if (id >= 1 && id <= MAX_MOTORS)
            {
                m0st_parse_status(d, &g_st[id - 1]);
                g_st_ok[id - 1] = g_st[id - 1].ck_ok;
            }
        }
    }
}

/* ================================================================
 *  电机控制命令
 * ================================================================ */

void can_motor_set_speed(uint8_t id, float rpm)         { m0st_set_speed(id, rpm); }
void can_motor_move_to(uint8_t id, float ang, float r)  { m0st_move_to(id, ang, r); }
void can_motor_move_abs(uint8_t id, float ang, int t, float r) { m0st_move_abs(id, ang, t, r); }
void can_motor_stop(uint8_t id)                         { m0st_stop(id); }
void can_motor_enable(uint8_t id, bool on)          { m0st_enable(id, on); }
void can_motor_query(uint8_t id, uint8_t sub)       { m0st_query(id, sub); }

/* ================================================================
 *  状态读取
 * ================================================================ */

static M0Stepper_Status *get_st(uint8_t id)
{
    if (id < 1 || id > MAX_MOTORS || !g_st_ok[id - 1]) return NULL;
    return &g_st[id - 1];
}

bool  can_motor_has_status(uint8_t id)        { return get_st(id) != NULL; }
float can_motor_get_angle(uint8_t id)          { M0Stepper_Status *s = get_st(id); return s ? s->angle_deg : 0.0f; }
float can_motor_get_speed(uint8_t id)          { M0Stepper_Status *s = get_st(id); return s ? s->rpm : 0.0f; }
int   can_motor_get_state(uint8_t id)          { M0Stepper_Status *s = get_st(id); return s ? s->state : -1; }
int   can_motor_get_turns(uint8_t id)          { M0Stepper_Status *s = get_st(id); return s ? s->turns : 0; }
bool  can_motor_is_done(uint8_t id)            { M0Stepper_Status *s = get_st(id); return s ? m0st_is_done(s) : false; }

bool can_motor_get_full_status(uint8_t id, M0Stepper_Status *st)
{
    M0Stepper_Status *s = get_st(id);
    if (!s) return false;
    *st = *s;
    return true;
}
