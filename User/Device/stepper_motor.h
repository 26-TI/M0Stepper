/**
 * @file stepper_motor.h
 * @brief 步进电机 STEP+DIR 驱动（器件层）
 *
 * 依赖：bsp_pwm.h（STEP 脉冲）、bsp_gpio.h（DIR 方向）
 * 与 MCU 无关，换平台只需 Bsp 层实现同名函数
 */

#ifndef __STEPPER_MOTOR_H__
#define __STEPPER_MOTOR_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STEPS_PER_REV   6400    ///< 200 × 32 细分（配合驱动器拨码）

void stepper_init(void);              ///< 初始化（停脉冲，DIR=正转）
void stepper_set_speed(float rpm);    ///< 设速度，>0=正转, <0=反转, |rpm|<0.5=停

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_MOTOR_H__ */
