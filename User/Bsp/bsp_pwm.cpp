/**
 * @file bsp_pwm.cpp
 * @brief BSP PWM — MSPM0 TIMG7 实现（STEP 脉冲）
 *
 * TIMG7 @ 1MHz (80MHz ÷ 80), PA26=CCP0=STEP
 * 50% 占空比，16 位定时器，频率通过重载值 period 控制
 */

#include "bsp_pwm.h"
#include "ti_msp_dl_config.h"

#define TIMER_CLK   1000000U   ///< 1MHz
#define MIN_PERIOD  10U        ///< 防止频率过高

void bsp_step_pwm_init(void)
{
    DL_TimerG_stopCounter(PWM_Step_INST);
}

void bsp_step_pwm_set_freq(uint32_t hz)
{
    if (hz == 0)
    {
        __disable_irq();
        DL_TimerG_stopCounter(PWM_Step_INST);
        __enable_irq();
        return;
    }

    uint32_t period = TIMER_CLK / hz;

    if (period > 65535U) period = 65535U;
    if (period < MIN_PERIOD) period = MIN_PERIOD;

    /* 关全局中断保护 TIMG7 寄存器原子更新，防止 UART1 ISR 竞争 */
    __disable_irq();
    DL_TimerG_setLoadValue(PWM_Step_INST, period);
    DL_TimerG_setCaptureCompareValue(PWM_Step_INST, period / 2,
                                      DL_TIMER_CC_0_INDEX);
    DL_TimerG_startCounter(PWM_Step_INST);
    __enable_irq();
}
