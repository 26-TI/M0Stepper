/**
 * @file motor_control.cpp
 * @brief 电机控制 — 速度开环 / 位置闭环 GOTO / 定时转动
 */

#include "motor_control.hpp"
#include "stepper_motor.h"
#include "speed_measure.h"

typedef enum { MODE_IDLE, MODE_SPEED, MODE_GOTO, MODE_TIMED } Mode;

static SpeedMeasure g_sm;
static Mode         g_mode          = MODE_IDLE;
static float        g_target_rpm    = 0.0f;
static float        g_last_angle    = 0.0f;
static int          g_total_turns   = 0;
static int          g_turns_init    = 0;

/* GOTO 参数 */
static float        g_goto_angle     = 0.0f;
static float        g_goto_abs       = 0.0f;
static float        g_goto_max_rpm   = 45.0f;
static float        g_goto_cur_rpm   = 0.0f;
static uint32_t     g_goto_done_ticks = 0;
#define GOTO_KP          2.0f
#define GOTO_ACCEL       10.0f
#define GOTO_MIN_RPM     5.0f
#define GOTO_STOP_THRES  0.15f
#define GOTO_BACK_THRES  1.0f
#define GOTO_DONE_TICKS  120

/* 定时转动 */
static uint32_t g_timed_ticks = 0;

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
    if (g_turns_init) {
        float d = enc->angle - g_last_angle;
        if      (d > 180.0f)  g_total_turns--;
        else if (d < -180.0f) g_total_turns++;
    } else g_turns_init = 1;
    g_last_angle = enc->angle;
}

void motor_tick(MT6816_Data *enc)
{
    /* ---- GOTO ---- */
    if (g_mode == MODE_GOTO)
    {
        float error = g_goto_abs - (g_total_turns * 360.0f + enc->angle);
        float rpm = -error * GOTO_KP;
        if (rpm > g_goto_max_rpm)   rpm =  g_goto_max_rpm;
        if (rpm < -g_goto_max_rpm)  rpm = -g_goto_max_rpm;
        float abs_err = (error > 0) ? error : -error;

        if (g_goto_done_ticks > 0) {
            g_goto_cur_rpm = 0.0f;
            stepper_set_speed(0.0f);
            g_target_rpm = 0.0f;
            if (abs_err < GOTO_BACK_THRES) g_goto_done_ticks++;
            else                           g_goto_done_ticks = 0;
        } else if (abs_err < GOTO_STOP_THRES) {
            g_goto_cur_rpm = 0.0f;
            stepper_set_speed(0.0f);
            g_target_rpm = 0.0f;
            g_goto_done_ticks = 1;
        } else {
            if (rpm > 0.0f && rpm < GOTO_MIN_RPM)  rpm = GOTO_MIN_RPM;
            if (rpm < 0.0f && rpm > -GOTO_MIN_RPM) rpm = -GOTO_MIN_RPM;
            float delta = rpm - g_goto_cur_rpm;
            if (delta > GOTO_ACCEL)  rpm = g_goto_cur_rpm + GOTO_ACCEL;
            if (delta < -GOTO_ACCEL) rpm = g_goto_cur_rpm - GOTO_ACCEL;
            g_goto_cur_rpm = rpm;
            g_target_rpm = rpm;
            stepper_set_speed(rpm);
        }
        if (g_goto_done_ticks >= GOTO_DONE_TICKS) g_mode = MODE_IDLE;
    }

    /* ---- 速度开环 ---- */
    if (g_mode == MODE_SPEED)
        stepper_set_speed(g_target_rpm);

    /* ---- 定时转动 ---- */
    if (g_mode == MODE_TIMED) {
        if (g_timed_ticks > 0) g_timed_ticks--;
        else g_mode = MODE_IDLE;
    }
}

/* ---- 速度指令 ---- */
void motor_set_speed(float rpm)    { g_mode = MODE_SPEED; g_target_rpm = rpm; }
void motor_stop(void)              { g_mode = MODE_IDLE; g_target_rpm = 0.0f; g_timed_ticks = 0; stepper_set_speed(0.0f); }

/* ---- 位置指令 ---- */
void motor_move_to(float angle_deg) { motor_move_to_ex(angle_deg, 45.0f); }

void motor_move_to_ex(float angle_deg, float max_rpm)
{
    float cur_abs = g_total_turns * 360.0f + g_last_angle;
    float target  = angle_deg;
    while (target - cur_abs > 180.0f)   target -= 360.0f;
    while (target - cur_abs < -180.0f)  target += 360.0f;
    int turns = (int)(target / 360.0f);
    if (target < 0) turns--;
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

int motor_move_done(void) { return g_goto_done_ticks >= GOTO_DONE_TICKS; }

void motor_timed_move(float angle_deg, float duration_s)
{
    float cur_abs = g_total_turns * 360.0f + g_last_angle;
    float target  = angle_deg;
    while (target - cur_abs > 180.0f)   target -= 360.0f;
    while (target - cur_abs < -180.0f)  target += 360.0f;
    float dist = target - cur_abs;
    float rpm  = dist / 360.0f * 60.0f / duration_s;
    g_mode        = MODE_TIMED;
    g_target_rpm  = rpm;
    g_timed_ticks = (uint32_t)(duration_s * 200.0f);
    stepper_set_speed(rpm);
}

int motor_timed_move_done(void) { return g_mode != MODE_TIMED; }

/* ---- 读取 ---- */
float motor_speed(void)         { return g_sm.actual_rpm; }
float motor_angle(void)         { return g_last_angle; }
int   motor_get_turns(void)     { return g_total_turns; }
float motor_target_speed(void)  { return g_target_rpm; }
int   motor_is_moving(void)     { return g_mode != MODE_IDLE; }
