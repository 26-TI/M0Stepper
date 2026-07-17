/**
 * @file bsp_pwm.h
 * @brief BSP PWM 抽象 — 步进电机 STEP 脉冲生成
 *
 * 换 MCU 时只需重写 bsp_pwm.cpp，接口保持不变
 */

#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_step_pwm_init(void);               ///< 初始化（停脉冲）
void bsp_step_pwm_set_freq(uint32_t hz);    ///< 设频率，0=停

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H__ */
