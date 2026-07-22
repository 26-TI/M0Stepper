/**
 * @file motor_control.hpp
 * @brief 步进电机控制类 — 速度开环 + 位置闭环 GOTO + 定时转动
 *
 * === 用法 ===
 *   MotorControl motor;
 *   motor.init();
 *   motor.measureTick(&enc);     // 1ms: 测速 + 圈数追踪
 *   motor.controlTick(&enc);     // 5ms: GOTO/速度/定时控制
 *   motor.setSpeed(120);
 *   motor.moveTo(90);
 */

#ifndef __MOTOR_CONTROL_HPP__
#define __MOTOR_CONTROL_HPP__

#include "mt6816.h"
#include "speed_measure.h"

class MotorControl
{
public:
    MotorControl() = default;
    void init();
    void measureTick(MT6816_Data *enc);  ///< 1ms: 测速 + 圈数追踪
    void controlTick(MT6816_Data *enc);  ///< 5ms: 控制逻辑

    /* ---- 速度 ---- */
    void setSpeed(float rpm);
    void stop();

    /* ---- 位置 ---- */
    void moveTo(float angle_deg, float max_rpm = 45.0f);      ///< 单圈就近
    void moveAbs(float angle_deg, int turns, float max_rpm);  ///< 绝对位置
    bool isDone() const;                                       ///< GOTO 到位?

    /* ---- 定时转动 ---- */
    void timedMove(float angle_deg, float duration_s);
    bool timedMoveDone() const;

    /* ---- 读取 ---- */
    float speed()        const { return sm_.actual_rpm; }
    float angle()        const { return lastAngle_; }
    int   turns()        const { return totalTurns_; }
    float targetSpeed()  const { return targetRpm_; }
    bool  isMoving()     const { return mode_ != IDLE; }

private:
    enum Mode { IDLE, SPEED, GOTO, TIMED };

    SpeedMeasure sm_;
    Mode         mode_          = IDLE;
    float        targetRpm_     = 0.0f;
    float        lastAngle_     = 0.0f;
    int          totalTurns_    = 0;
    bool         turnsInit_     = false;

    /* GOTO */
    float        gotoAbs_       = 0.0f;
    float        gotoMaxRpm_    = 45.0f;
    float        gotoCurRpm_    = 0.0f;
    uint32_t     gotoDoneTicks_ = 0;

    /* TIMED */
    uint32_t     timedTicks_    = 0;

    static constexpr float KP          = 2.0f;
    static constexpr float ACCEL       = 10.0f;
    static constexpr float MIN_RPM     = 5.0f;
    static constexpr float STOP_THRES  = 0.15f;
    static constexpr float BACK_THRES  = 1.0f;
    static constexpr uint32_t DONE_TICKS = 120;
};

#endif
