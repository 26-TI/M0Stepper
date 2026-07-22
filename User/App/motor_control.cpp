/**
 * @file motor_control.cpp
 * @brief 步进电机控制类实现
 */

#include "motor_control.hpp"
#include "stepper_motor.h"
#include <math.h>

MotorControl::MotorControl() {}

void MotorControl::init()
{
    stepper_init();
    speed_measure_init(&sm_);
    mode_      = IDLE;
    targetRpm_ = 0.0f;
}

void MotorControl::measureTick(MT6816_Data *enc)
{
    speed_measure_update(&sm_, enc);

    /* 圈数追踪 */
    if (turnsInit_) {
        float d = enc->angle - lastAngle_;
        if      (d > 180.0f) totalTurns_--;
        else if (d < -180.0f) totalTurns_++;
    } else {
        turnsInit_ = true;
    }
    lastAngle_ = enc->angle;
}

void MotorControl::controlTick(MT6816_Data *enc)
{
    switch (mode_)
    {
    case GOTO:
    {
        float error = gotoAbs_ - (totalTurns_ * 360.0f + enc->angle);
        float rpm = -error * KP;
        if (rpm > gotoMaxRpm_)  rpm =  gotoMaxRpm_;
        if (rpm < -gotoMaxRpm_) rpm = -gotoMaxRpm_;

        float absErr = fabsf(error);

        if (gotoDoneTicks_ > 0) {
            /* 已进入停转区，保持停止防抖 */
            gotoCurRpm_ = 0.0f;
            stepper_set_speed(0.0f);
            targetRpm_ = 0.0f;

            if (absErr < BACK_THRES)
                gotoDoneTicks_++;
            else
                gotoDoneTicks_ = 0;
        }
        else if (absErr < STOP_THRES) {
            /* 初次进入停转区 */
            gotoCurRpm_ = 0.0f;
            stepper_set_speed(0.0f);
            targetRpm_ = 0.0f;
            gotoDoneTicks_ = 1;
        }
        else {
            /* 正常 P 接近 + 加减速平滑 */
            if (rpm > 0.0f && rpm < MIN_RPM)  rpm = MIN_RPM;
            if (rpm < 0.0f && rpm > -MIN_RPM) rpm = -MIN_RPM;

            float delta = rpm - gotoCurRpm_;
            if (delta > ACCEL)  rpm = gotoCurRpm_ + ACCEL;
            if (delta < -ACCEL) rpm = gotoCurRpm_ - ACCEL;
            gotoCurRpm_ = rpm;

            targetRpm_ = rpm;
            stepper_set_speed(rpm);
        }

        if (gotoDoneTicks_ >= DONE_TICKS)
            mode_ = IDLE;
        break;
    }

    case SPEED:
        stepper_set_speed(targetRpm_);
        break;

    case TIMED:
        if (timedTicks_ > 0)
            timedTicks_--;
        else
            mode_ = IDLE;
        break;

    default:
        break;
    }
}

/* ================================================================
 *  速度指令
 * ================================================================ */

void MotorControl::setSpeed(float rpm)
{
    mode_      = SPEED;
    targetRpm_ = rpm;
}

void MotorControl::stop()
{
    mode_       = IDLE;
    targetRpm_  = 0.0f;
    timedTicks_ = 0;
    stepper_set_speed(0.0f);
}

/* ================================================================
 *  位置指令
 * ================================================================ */

void MotorControl::moveTo(float angle_deg, float max_rpm)
{
    /* 就近走，最多 180° */
    float curAbs = totalTurns_ * 360.0f + lastAngle_;
    float target = angle_deg;
    while (target - curAbs > 180.0f)  target -= 360.0f;
    while (target - curAbs < -180.0f) target += 360.0f;
    int turns = (int)(target / 360.0f);
    if (target < 0) turns--;
    moveAbs(angle_deg, turns, max_rpm);
}

void MotorControl::moveAbs(float angle_deg, int turns, float max_rpm)
{
    gotoAbs_       = turns * 360.0f + angle_deg;
    gotoMaxRpm_    = max_rpm > 0 ? max_rpm : 45.0f;
    gotoDoneTicks_ = 0;
    gotoCurRpm_    = 0.0f;
    mode_ = GOTO;
}

bool MotorControl::isDone() const
{
    return gotoDoneTicks_ >= DONE_TICKS;
}

/* ================================================================
 *  定时转动
 * ================================================================ */

void MotorControl::timedMove(float angle_deg, float duration_s)
{
    /* 就近计算距离 */
    float curAbs = totalTurns_ * 360.0f + lastAngle_;
    float target = angle_deg;
    while (target - curAbs > 180.0f)  target -= 360.0f;
    while (target - curAbs < -180.0f) target += 360.0f;
    float dist = target - curAbs;

    /* RPM = 距离(°) / 360 × 60 / 时间(s) */
    float rpm = dist / 360.0f * 60.0f / duration_s;

    mode_       = TIMED;
    targetRpm_  = rpm;
    timedTicks_ = (uint32_t)(duration_s * 200.0f);
    stepper_set_speed(rpm);
}

bool MotorControl::timedMoveDone() const
{
    return mode_ != TIMED;
}
