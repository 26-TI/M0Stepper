/**
 * @file motor_control.hpp
 * @brief 电机控制 — 应用层统一接口
 *
 * === 速度控制（开环）===
 *   motor_set_speed(120)       // 120 RPM
 *   motor_set_speed(-60)       // 60 RPM 反转
 *   motor_stop()               // 停
 *
 * === 位置控制（编码器闭环）===
 *   motor_move_to(90)          // 单圈就近到 90°（限速 45）
 *   motor_move_to_ex(180, 60)  // 单圈就近到 180° 限速 60
 *   motor_move_abs(90, 3, 60)  // 绝对：第3圈 90° 限速 60
 *   motor_move_done()          // true=到位
 *
 * === 读取 ===
 *   motor_speed()              // 实测 RPM
 *   motor_angle()              // 编码器角度 °
 *   motor_target_speed()       // 目标 RPM
 *
 * === 用法 ===
 *   motor_init();
 *   while (1) {
 *       encoder_read(&enc);
 *       motor_tick(&enc);
 *   }
 */

#ifndef __MOTOR_CONTROL_HPP__
#define __MOTOR_CONTROL_HPP__

#include "mt6816.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 生命周期 ---- */
void motor_init(void);
void motor_tick(MT6816_Data *enc);          ///< 控制逻辑, 5ms 周期
void motor_measure_tick(MT6816_Data *enc);  ///< 测速 + 角度 + 圈数跟踪, 1ms 周期

/* ---- 速度指令（开环） ---- */
void motor_set_speed(float rpm);
void motor_stop(void);

/* ---- 位置指令（编码器闭环） ---- */
void motor_move_to(float angle_deg);                                   ///< 单圈就近 GOTO
void motor_move_to_ex(float angle_deg, float max_rpm);                 ///< 单圈就近 GOTO, 指定限速
void motor_move_abs(float angle_deg, int turns, float max_rpm);        ///< 绝对位置 GOTO
int  motor_move_done(void);

/* ---- 读取 ---- */
float motor_speed(void);
float motor_angle(void);
int   motor_get_turns(void);            ///< 累计圈数
float motor_target_speed(void);
int   motor_is_moving(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_HPP__ */
