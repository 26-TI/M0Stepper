/**
 * @file pid.cpp
 * @brief 通用 PID 控制器 — 位置式 + 积分抗饱和 + 输出限幅
 */

#include "pid.h"

void pid_init(PID *pid, float kp, float ki, float kd, float ilim, float olim)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = ilim;
    pid->output_limit   = olim;
}

float pid_update(PID *pid, float target, float actual, float dt)
{
    float error = target - actual;

    /* ---- 比例项 ---- */
    float p_term = pid->kp * error;

    /* ---- 积分项（带抗饱和） ---- */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    float i_term = pid->ki * pid->integral;

    /* ---- 微分项 ---- */
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error  = error;
    float d_term = pid->kd * derivative;

    /* ---- 合成输出 + 限幅 + back-calculation 抗饱和 ---- */
    float output = p_term + i_term + d_term;

    if (output > pid->output_limit)
    {
        /* 超上限：反算积分，使输出恰好 = output_limit */
        if (pid->ki > 0.0f)
            pid->integral -= (output - pid->output_limit) / pid->ki;
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit)
    {
        if (pid->ki > 0.0f)
            pid->integral -= (output + pid->output_limit) / pid->ki;
        output = -pid->output_limit;
    }

    return output;
}
