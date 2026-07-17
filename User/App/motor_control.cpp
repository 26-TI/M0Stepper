/**
 * @file motor_control.cpp
 * @brief 电机控制 — 应用层实现（速度开环 / 位置闭环 / GOTO）
 */

#include "motor_control.hpp"
#include "stepper_motor.h"
#include "speed_measure.h"

/* ---- 内部模式 ---- */
typedef enum { MODE_IDLE, MODE_SPEED, MODE_GOTO } Mode;

static SpeedMeasure g_sm;
static Mode         g_mode        = MODE_IDLE;
static float     g_target_rpm  = 0.0f;
static float     g_last_angle  = 0.0f;

/* ---- GOTO 参数 ---- */
static float     g_goto_angle     = 0.0f;
static float     g_goto_max_rpm   = 45.0f;
static uint32_t  g_goto_done_ticks = 0;
#define GOTO_KP          2.0f
#define GOTO_DONE_THRES  1.0f     ///< 到位角度阈值 (°)
#define GOTO_DONE_TICKS  100      ///< 持续 100ms

/* ================================================================
 *  公开 API
 * ================================================================ */

void motor_init(void)
{
    stepper_init();
    speed_measure_init(&g_sm);
    g_mode       = MODE_IDLE;
    g_target_rpm = 0.0f;
}

void motor_measure_tick(MT6816_Data *enc)
{
    /* 测速 + 角度记录 — 纯计算，不碰硬件，可高频 1ms 调用 */
    speed_measure_update(&g_sm, enc);
    g_last_angle = enc->angle;
}

void motor_tick(MT6816_Data *enc)
{
    /* ---- GOTO：P 直驱，不经过速度 PID ---- */
    if (g_mode == MODE_GOTO)
    {
        float error = g_goto_angle - enc->angle;
        if (error > 180.0f)  error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        float rpm = -error * GOTO_KP;
        if (rpm > g_goto_max_rpm)   rpm =  g_goto_max_rpm;
        if (rpm < -g_goto_max_rpm)  rpm = -g_goto_max_rpm;

        float abs_err = (error > 0) ? error : -error;
        if (abs_err < GOTO_DONE_THRES)
            g_goto_done_ticks++;
        else
            g_goto_done_ticks = 0;

        if (g_goto_done_ticks >= GOTO_DONE_TICKS)
        {
            stepper_set_speed(0.0f);
            g_target_rpm = 0.0f;
        }
        else if (abs_err < GOTO_DONE_THRES)
        {
            /* 已进入阈值区，停转等 settling */
            stepper_set_speed(0.0f);
            g_target_rpm = 0.0f;
        }
        else
        {
            /* 最低速保底 */
            if (rpm > 0.0f && rpm < 3.0f)  rpm = 3.0f;
            if (rpm < 0.0f && rpm > -3.0f) rpm = -3.0f;
            g_target_rpm = rpm;
            stepper_set_speed(rpm);
        }
    }

    /* ---- 开环速度 ---- */
    if (g_mode == MODE_SPEED)
        stepper_set_speed(g_target_rpm);
}

/* ---- 速度指令 ---- */

void motor_set_speed(float rpm)
{
    g_mode       = MODE_SPEED;
    g_target_rpm = rpm;
}

void motor_stop(void)
{
    g_mode       = MODE_IDLE;
    g_target_rpm = 0.0f;
    stepper_set_speed(0.0f);
}

/* ---- 位置指令 ---- */

void motor_move_to(float angle_deg)
{
    motor_move_to_ex(angle_deg, 45.0f);
}

void motor_move_to_ex(float angle_deg, float max_rpm)
{
    g_goto_angle     = angle_deg;
    g_goto_max_rpm   = max_rpm;
    g_goto_done_ticks = 0;
    g_mode = MODE_GOTO;
}

int motor_move_done(void)
{
    return g_goto_done_ticks >= GOTO_DONE_TICKS;
}

/* ---- 读取 ---- */

float motor_speed(void)         { return g_sm.actual_rpm; }
float motor_angle(void)         { return g_last_angle; }
float motor_target_speed(void)  { return g_target_rpm; }
int   motor_is_moving(void)     { return g_mode != MODE_IDLE; }
