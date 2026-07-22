/**
 * @file motor_control.cpp
 * @brief 电机控制 — 应用层实现（速度开环 / 位置闭环 / GOTO）
 */

#include "motor_control.hpp"
#include "stepper_motor.h"
#include "speed_measure.h"

/* ---- 内部模式 ---- */
typedef enum { MODE_IDLE, MODE_SPEED, MODE_GOTO, MODE_TIMED } Mode;

static SpeedMeasure g_sm;
static Mode         g_mode          = MODE_IDLE;
static float     g_target_rpm    = 0.0f;
static float     g_last_angle    = 0.0f;
static int       g_total_turns   = 0;
static int       g_turns_init    = 0;

/* ---- GOTO 参数 ---- */
static float     g_goto_angle     = 0.0f;
static float     g_goto_abs       = 0.0f;
static float     g_goto_max_rpm   = 45.0f;
static float     g_goto_cur_rpm   = 0.0f;   /* 当前平滑后转速 */
static uint32_t  g_goto_done_ticks = 0;
#define GOTO_KP          2.0f
#define GOTO_ACCEL       10.0f     /* 每 5ms 最大转速变化 = 2000 RPM/s² */
#define GOTO_MIN_RPM     5.0f      /* 最低转速 */
#define GOTO_STOP_THRES  0.15f     /* 接近目标时停转 */
#define GOTO_BACK_THRES  1.0f      /* 停住后漂出 1° 才重调 */
#define GOTO_DONE_TICKS  120       /* 稳定 600ms 判定到位 */

/* ---- 定时转动参数 ---- */
static uint32_t  g_timed_ticks = 0;

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
    /* ---- GOTO：位置闭环 P 控制 ---- */
    if (g_mode == MODE_GOTO)
    {
        float error = g_goto_abs - (g_total_turns * 360.0f + enc->angle);

        float rpm = -error * GOTO_KP;
        if (rpm > g_goto_max_rpm)   rpm =  g_goto_max_rpm;
        if (rpm < -g_goto_max_rpm)  rpm = -g_goto_max_rpm;

        float abs_err = (error > 0) ? error : -error;

        if (g_goto_done_ticks > 0)
        {
            /* ---- 已进入停转区，保持停止，防抖 ---- */
            g_goto_cur_rpm = 0.0f;
            stepper_set_speed(0.0f);
            g_target_rpm = 0.0f;

            if (abs_err < GOTO_BACK_THRES)
                g_goto_done_ticks++;        /* 稳定，继续累计 */
            else
                g_goto_done_ticks = 0;      /* 漂出 1°，重新接近 */
        }
        else if (abs_err < GOTO_STOP_THRES)
        {
            /* ---- 初次进入停转区，立刻停 ---- */
            g_goto_cur_rpm = 0.0f;
            stepper_set_speed(0.0f);
            g_target_rpm = 0.0f;
            g_goto_done_ticks = 1;
        }
        else
        {
            /* ---- 正常 P 接近，最低 5 RPM + 加减速平滑 ---- */
            if (rpm > 0.0f && rpm < GOTO_MIN_RPM)  rpm = GOTO_MIN_RPM;
            if (rpm < 0.0f && rpm > -GOTO_MIN_RPM) rpm = -GOTO_MIN_RPM;

            /* 加速度限制，防转速突变 */
            float delta = rpm - g_goto_cur_rpm;
            if (delta > GOTO_ACCEL)  rpm = g_goto_cur_rpm + GOTO_ACCEL;
            if (delta < -GOTO_ACCEL) rpm = g_goto_cur_rpm - GOTO_ACCEL;
            g_goto_cur_rpm = rpm;

            g_target_rpm = rpm;
            stepper_set_speed(rpm);
        }

        if (g_goto_done_ticks >= GOTO_DONE_TICKS)
            g_mode = MODE_IDLE;
    }

    /* ---- 速度开环 ---- */
    if (g_mode == MODE_SPEED)
        stepper_set_speed(g_target_rpm);

    /* ---- 定时转动 ---- */
    if (g_mode == MODE_TIMED)
    {
        if (g_timed_ticks > 0)
            g_timed_ticks--;
        else
            g_mode = MODE_IDLE;  /* 时间到 */
    }
}

/* ---- 速度指令 ---- */

void motor_set_speed(float rpm)
{
    g_mode       = MODE_SPEED;
    g_target_rpm = rpm;
}

void motor_stop(void)
{
    g_mode         = MODE_IDLE;
    g_target_rpm   = 0.0f;
    g_timed_ticks  = 0;
    stepper_set_speed(0.0f);
}

/* ---- 位置指令 ---- */

void motor_move_to(float angle_deg)
{
    motor_move_to_ex(angle_deg, 45.0f);
}

void motor_move_to_ex(float angle_deg, float max_rpm)
{
    /* 单圈 GOTO：找最近的等效位置，不绕远路 */
    float cur_abs = g_total_turns * 360.0f + g_last_angle;
    float target  = angle_deg;
    /* 找最接近当前位置的等效角度（加减整数圈） */
    while (target - cur_abs > 180.0f)   target -= 360.0f;
    while (target - cur_abs < -180.0f)  target += 360.0f;
    int turns = (int)(target / 360.0f);
    if (target < 0) turns--;  /* floor division for negative */
    motor_move_abs(angle_deg, turns, max_rpm);
}

void motor_move_abs(float angle_deg, int turns, float max_rpm)
{
    g_goto_angle     = angle_deg;
    g_goto_abs       = turns * 360.0f + angle_deg;
    g_goto_max_rpm   = max_rpm > 0 ? max_rpm : 45.0f;
    g_goto_done_ticks = 0;
    g_goto_cur_rpm    = 0.0f;
    g_mode = MODE_GOTO;
}

int motor_move_done(void)
{
    return g_goto_done_ticks >= GOTO_DONE_TICKS;
}

void motor_timed_move(float angle_deg, float duration_s)
{
    /* 就近计算距离 */
    float cur_abs = g_total_turns * 360.0f + g_last_angle;
    float target  = angle_deg;
    while (target - cur_abs > 180.0f)   target -= 360.0f;
    while (target - cur_abs < -180.0f)  target += 360.0f;
    float dist = target - cur_abs;  /* 带符号角度差 */

    /* RPM = 距离(°) / 360 × 60 / 时间(s) */
    float rpm = dist / 360.0f * 60.0f / duration_s;

    g_mode        = MODE_TIMED;
    g_target_rpm  = rpm;
    g_timed_ticks = (uint32_t)(duration_s * 200.0f);  /* 200 ticks/s (5ms周期) */

    stepper_set_speed(rpm);
}

int motor_timed_move_done(void)
{
    return g_mode != MODE_TIMED;
}

/* ---- 读取 ---- */

float motor_speed(void)         { return g_sm.actual_rpm; }
float motor_angle(void)         { return g_last_angle; }
int   motor_get_turns(void)     { return g_total_turns; }
float motor_target_speed(void)  { return g_target_rpm; }
int   motor_is_moving(void)     { return g_mode != MODE_IDLE; }
