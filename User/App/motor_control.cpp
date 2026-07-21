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
static Mode         g_mode          = MODE_IDLE;
static float     g_target_rpm    = 0.0f;
static float     g_last_angle    = 0.0f;
static int       g_total_turns   = 0;       // 累计圈数
static int       g_turns_init    = 0;       // 第一次标记

/* ---- GOTO 参数 ---- */
static float     g_goto_angle     = 0.0f;   // 单圈目标角度
static float     g_goto_abs       = 0.0f;   // 多圈绝对目标 (°)
static float     g_goto_max_rpm   = 45.0f;
static uint32_t  g_goto_done_ticks = 0;
#define GOTO_KP          2.0f
#define GOTO_DONE_THRES  1.0f
#define GOTO_DONE_TICKS  100

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
    speed_measure_update(&g_sm, enc);

    /* 圈数追踪：检测过零点 */
    if (g_turns_init)
    {
        float d = enc->angle - g_last_angle;
        if      (d > 180.0f) g_total_turns--;
        else if (d < -180.0f) g_total_turns++;
    }
    else g_turns_init = 1;

    g_last_angle = enc->angle;
}

void motor_tick(MT6816_Data *enc)
{
    /* ---- GOTO：单圈用短弧，多圈走全路径 ---- */
    if (g_mode == MODE_GOTO)
    {
        float error = g_goto_abs - (g_total_turns * 360.0f + enc->angle);

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
            stepper_set_speed(0.0f);
            g_target_rpm = 0.0f;
        }
        else
        {
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
    motor_move_to_multi(angle_deg, 0, max_rpm);
}

void motor_move_to_multi(float angle_deg, int turns, float max_rpm)
{
    g_goto_angle     = angle_deg;
    g_goto_abs       = turns * 360.0f + angle_deg;
    g_goto_max_rpm   = max_rpm > 0 ? max_rpm : 45.0f;
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
int   motor_get_turns(void)     { return g_total_turns; }
float motor_target_speed(void)  { return g_target_rpm; }
int   motor_is_moving(void)     { return g_mode != MODE_IDLE; }
