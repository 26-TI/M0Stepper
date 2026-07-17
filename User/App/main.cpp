/**
 * @file main.cpp
 * @brief 步进电机角度闭环控制 + 陀螺仪数据
 * @date 2026-07-16
 *
 * 序列：0° → 90° → 180° → 270° → 循环
 */

#include "main.hpp"
#include "ssd1306.h"
#include "motor_control.hpp"
#include "bsp_gpio.h"
#include "bsp_gyro.h"
#include "bsp_imu601.h"

static volatile bool     g_control_flag = false;
static volatile uint32_t g_tick         = 0;

static const float g_angle_seq[] = { 0.0f, 90.0f, 180.0f, 270.0f };
static int         g_seq_idx      = 0;
static float       g_target_angle = 0.0f;

extern "C" void TIMA0_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_TICK_INST))
    {
        case DL_TIMERA_IIDX_ZERO:
            g_control_flag = true;
            g_tick++;
            break;
        default: break;
    }
}

int main()
{
    SYSCFG_DL_init();

    ssd1306_init();
    ssd1306_printf(0, 0, "  Angle Control");
    ssd1306_printf(0, 2, "  Goto + Gyro");

    mt6816_init();
    motor_init();
    bsp_gyro_init();
    bsp_imu601_init();

    /* 使能 1ms 定时器 — 放在所有初始化之后 */
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);

    g_target_angle = g_angle_seq[0];
    motor_move_to_ex(g_target_angle, 60.0f);

    bsp_uart_printf("=== Angle Control + Gyro ===\r\n");

    while (1)
    {
        if (g_control_flag)
        {
            g_control_flag = false;

            /* 编码器读取 + 测速/角度记录：1ms，不碰硬件 */
            MT6816_Data enc;
            mt6816_read(&enc);
            motor_measure_tick(&enc);

            /* 两个陀螺仪数据消费 */
            bsp_gyro_poll();
            bsp_imu601_poll();

            /* 电机 PWM 控制：5ms，自保护关中断 */
            if ((g_tick % 5) == 0)
                motor_tick(&enc);

            /* 到位后停留 500ms 再切下一个 */
            static uint32_t done_hold = 0;
            if (motor_move_done())
            {
                if (++done_hold > 500)
                {
                    done_hold = 0;
                    g_seq_idx = (g_seq_idx + 1) % 4;
                    g_target_angle = g_angle_seq[g_seq_idx];
                    motor_move_to_ex(g_target_angle, 60.0f);
                }
            }
            else
                done_hold = 0;

            if ((g_tick % 200) == 0)
            {
                float ang = motor_angle();
                float err = g_target_angle - ang;
                if (err > 180.0f)  err -= 360.0f;
                if (err < -180.0f) err += 360.0f;

                float gy_yaw = bsp_gyro_get_yaw();
                float gy_wz  = bsp_gyro_get_wz();
                float im_yaw   = bsp_imu601_get_yaw();
                float im_pitch = bsp_imu601_get_pitch();
                float im_roll  = bsp_imu601_get_roll();

                /* OLED: WT901 行0, IMU601 行2, 电机 行4+6 */
                ssd1306_printf(0, 0, "G%4d W%4d",
                    (int)gy_yaw, (int)gy_wz);
                ssd1306_printf(0, 2, "Y%4d P%4d R%4d",
                    (int)im_yaw, (int)im_pitch, (int)im_roll);
                ssd1306_printf(0, 4, "Ang %4d T%3d",
                    (int)ang, (int)g_target_angle);
                ssd1306_printf(0, 6, "Spd%3d Err%3d",
                    (int)motor_speed(), (int)err);

                bsp_uart_printf("ang:%d tgt:%d err:%d spd:%d "
                    "G_y:%d G_w:%d rx_g:%lu "
                    "I_y:%d I_p:%d I_r:%d rx_i:%lu\r\n",
                    (int)ang, (int)g_target_angle, (int)err, (int)motor_speed(),
                    (int)gy_yaw, (int)gy_wz,
                    (unsigned long)bsp_gyro_rx_total(),
                    (int)im_yaw, (int)im_pitch, (int)im_roll,
                    (unsigned long)bsp_imu601_rx_total());

                bsp_led_toggle();
            }
        }
    }
}
