/**
 * @file motor_control.cpp
 * @brief 步进电机控制类 — 速度开环 / 位置闭环 GOTO / 定时转动
 */

#include "motor_control.hpp"
#include "stepper_motor.h"
#include "speed_measure.h"

extern StepperMotor stepper_motor;

/* ================================================================
 *  初始化
 * ================================================================ */

void MotorControl::init()
{
    speed_measure_init(&sm_);
    mode_      = IDLE;
    targetRpm_ = 0.0f;
}

/* ================================================================
 *  measureTick — 每 1ms: 测速 + 圈数追踪
 * ================================================================ */

void MotorControl::measureTick(MT6816_Data *enc)
{
    speed_measure_update(&sm_, enc);

    if (turnsInit_) {
        float d = enc->angle - lastAngle_;
        if      (d >  180.0f) totalTurns_--;
        else if (d < -180.0f) totalTurns_++;
    } else {
        turnsInit_ = true;
    }
    lastAngle_ = enc->angle;
}

/* ================================================================
 *  controlTick — 每 5ms: GOTO / SPEED / TIMED 控制
 * ================================================================ */

void MotorControl::controlTick(MT6816_Data *enc)
{
    /* ---- GOTO ---- */
    if (mode_ == GOTO)
    {
        float cur_abs = totalTurns_ * 360.0f + enc->angle;
        float error   = gotoAbs_ - cur_abs;
        float rpm     = -error * KP;
        if (rpm >  gotoMaxRpm_) rpm =  gotoMaxRpm_;
        if (rpm < -gotoMaxRpm_) rpm = -gotoMaxRpm_;
        float abs_err = (error > 0) ? error : -error;

        if (gotoDoneTicks_ > 0) {
            gotoCurRpm_ = 0.0f;
            stepper_motor.set_speed(0.0f);
            targetRpm_ = 0.0f;
            if (abs_err < BACK_THRES) gotoDoneTicks_++;
            else                      gotoDoneTicks_ = 0;
        } else if (abs_err < STOP_THRES) {
            gotoCurRpm_ = 0.0f;
            stepper_motor.set_speed(0.0f);
            targetRpm_ = 0.0f;
            gotoDoneTicks_ = 1;
        } else {
            if (rpm > 0.0f && rpm < MIN_RPM)  rpm = MIN_RPM;
            if (rpm < 0.0f && rpm > -MIN_RPM) rpm = -MIN_RPM;
            float delta = rpm - gotoCurRpm_;
            if (delta >  ACCEL) rpm = gotoCurRpm_ + ACCEL;
            if (delta < -ACCEL) rpm = gotoCurRpm_ - ACCEL;
            gotoCurRpm_ = rpm;
            targetRpm_  = rpm;
            stepper_motor.set_speed(rpm);
        }
        if (gotoDoneTicks_ >= DONE_TICKS) mode_ = IDLE;
    }

    /* ---- 速度开环 ---- */
    if (mode_ == SPEED)
        stepper_motor.set_speed(targetRpm_);

    /* ---- 定时转动 ---- */
    if (mode_ == TIMED) {
        if (timedTicks_ > 0) timedTicks_--;
        else mode_ = IDLE;
    }
}

/* ================================================================
 *  速度指令
 * ================================================================ */

void MotorControl::setSpeed(float rpm) { mode_ = SPEED; targetRpm_ = rpm; }
void MotorControl::stop()             { mode_ = IDLE; targetRpm_ = 0.0f; timedTicks_ = 0; stepper_motor.set_speed(0.0f); }

/* ================================================================
 *  位置指令
 * ================================================================ */

void MotorControl::moveTo(float angle_deg, float max_rpm)
{
    float cur_abs = totalTurns_ * 360.0f + lastAngle_;
    float target  = angle_deg;
    while (target - cur_abs >  180.0f) target -= 360.0f;
    while (target - cur_abs < -180.0f) target += 360.0f;
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
    mode_          = GOTO;
}

bool MotorControl::isDone() const { return gotoDoneTicks_ >= DONE_TICKS; }

void MotorControl::timedMove(float angle_deg, float duration_s)
{
    float cur_abs = totalTurns_ * 360.0f + lastAngle_;
    float target  = angle_deg;
    while (target - cur_abs >  180.0f) target -= 360.0f;
    while (target - cur_abs < -180.0f) target += 360.0f;
    float dist = target - cur_abs;
    float rpm  = dist / 360.0f * 60.0f / duration_s;
    mode_        = TIMED;
    targetRpm_   = rpm;
    timedTicks_  = (uint32_t)(duration_s * 200.0f);
    stepper_motor.set_speed(rpm);
}

bool MotorControl::timedMoveDone() const { return mode_ != TIMED; }
