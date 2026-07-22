/**
 * @file motor_control.hpp
 * @brief 电机控制 — 应用层统一接口
 */

#ifndef __MOTOR_CONTROL_HPP__
#define __MOTOR_CONTROL_HPP__

#include "mt6816.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 生命周期 ---- */
void motor_init(void);
void motor_tick(MT6816_Data *enc);
void motor_measure_tick(MT6816_Data *enc);

/* ---- 速度指令（开环） ---- */
void motor_set_speed(float rpm);
void motor_stop(void);

/* ---- 位置指令（编码器闭环） ---- */
void motor_move_to(float angle_deg);
void motor_move_to_ex(float angle_deg, float max_rpm);
void motor_move_abs(float angle_deg, int turns, float max_rpm);
void motor_timed_move(float angle_deg, float duration_s);
int  motor_move_done(void);
int  motor_timed_move_done(void);

/* ---- 读取 ---- */
float motor_speed(void);
float motor_angle(void);
int   motor_get_turns(void);
float motor_target_speed(void);
int   motor_is_moving(void);

#ifdef __cplusplus
}
#endif

#endif
