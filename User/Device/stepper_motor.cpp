/**
 * @file stepper_motor.cpp
 * @brief 步进电机 STEP+DIR — 使用 Bsp 层 PWM + GPIO
 *
 * 速度公式：step_freq = RPM × STEPS_PER_REV / 60
 * Bsp 层负责硬件 PWM 生成，本层只做数学换算
 */

#include "stepper_motor.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"

#define STEPS_PER_REV   6400

void stepper_init(void)
{
    bsp_dir_high();
    bsp_step_pwm_init();
}

void stepper_set_speed(float rpm)
{
    /* 方向 */
    if (rpm >= 0.0f)
        bsp_dir_high();
    else {
        bsp_dir_low();
        rpm = -rpm;
    }

    /* 停止 */
    if (rpm < 0.5f) {
        bsp_step_pwm_set_freq(0);
        return;
    }

    /* RPM → 脉冲频率(Hz) */
    uint32_t freq = (uint32_t)(rpm * (float)STEPS_PER_REV / 60.0f);
    bsp_step_pwm_set_freq(freq);
}
