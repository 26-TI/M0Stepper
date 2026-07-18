/**
 * @file main.cpp
 * @brief FreeRTOS 步进电机角度闭环 + 双陀螺仪 + OLED 菜单
 * @date 2026-07-18
 */

#include "main.hpp"
#include "ssd1306.h"
#include "motor_control.hpp"
#include "bsp_gpio.h"
#include "bsp_gyro.h"
#include "bsp_imu601.h"
#include "menu_ui.h"

#include "FreeRTOS.h"
#include "task.h"

/* ================================================================
 *  菜单定义 — 你的电赛题目
 * ================================================================ */
static MenuEntry g_menu[] = {
    { "Q1: Angle Goto",  "0->90->180->270"   },
    { "Q2: Speed Ctrl",  "Open Loop 60RPM"   },
    { "Q3: Sensor View", "Gyro + IMU601"     },
    { "Q4: PID Test",    "Position P Loop"   },
    { "Q5: Free",        "Custom Test"       },
};
static const int g_menu_count = sizeof(g_menu) / sizeof(g_menu[0]);
static Menu g_menu_state;
static MenuUI g_menu_ui;

/* ================================================================
 *  题目回调：每个题目有 init（进题时调用一次）和 display（每帧显示）
 *  题目函数可以读 motor_angle() / bsp_gyro_get_yaw() 等在控制任务里更新的数据
 * ================================================================ */

/* ---- 题目辅助：退出时停电机 ---- */
static void stop_motor(void)  { motor_stop(); }
static void noop(void)        {}

/* ---- Q1: 角度序列 ---- */
static float g_angle_seq[]   = { 0.0f, 90.0f, 180.0f, 270.0f };
static int   g_seq_idx       = 0;
static float g_target_angle  = 0.0f;

static void q1_init(void)
{
    g_seq_idx      = 0;
    g_target_angle = g_angle_seq[0];
    motor_move_to_ex(g_target_angle, 60.0f);
}

static void q1_display(void)
{
    float ang = motor_angle();
    float err = g_target_angle - ang;
    if (err > 180.0f)  err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    int spd = (int)motor_speed();

    ssd1306_printf(0, 0, "Q1: Angle Goto");
    ssd1306_printf(0, 2, "Cur:%3d Tgt:%3d", (int)ang, (int)g_target_angle);
    ssd1306_printf(0, 4, "Spd:%3d Err:%3d", spd, (int)err);
    ssd1306_printf(0, 6, "OK: Back");

    /* 到位后停 500ms 切下一个 */
    static uint32_t done_hold = 0;
    if (motor_move_done())
    {
        if (++done_hold > 500 / 50)   /* 50ms 周期 → 500ms */
        {
            done_hold = 0;
            g_seq_idx = (g_seq_idx + 1) % 4;
            g_target_angle = g_angle_seq[g_seq_idx];
            motor_move_to_ex(g_target_angle, 60.0f);
        }
    }
    else
        done_hold = 0;
}

/* ---- Q2: 速度控制 ---- */
static void q2_init(void)    { motor_set_speed(60); }
static void q2_display(void)
{
    ssd1306_printf(0, 0, "Q2: Speed Ctrl");
    ssd1306_printf(0, 2, "Speed: %3d RPM", (int)motor_speed());
    ssd1306_printf(0, 4, "Angle: %3d",     (int)motor_angle());
    ssd1306_printf(0, 6, "OK: Back");
}

/* ---- Q3: 双陀螺仪读数 ---- */
static void q3_init(void) {}
static void q3_display(void)
{
    ssd1306_printf(0, 0, "WT Y%3d W%3d",
                   (int)bsp_gyro_get_yaw(), (int)bsp_gyro_get_wz());
    ssd1306_printf(0, 2, "IMU Y%3d P%3d R%3d",
                   (int)bsp_imu601_get_yaw(),
                   (int)bsp_imu601_get_pitch(),
                   (int)bsp_imu601_get_roll());
    ssd1306_printf(0, 4, "Rx G:%lu I:%lu",
                   (unsigned long)bsp_gyro_rx_total(),
                   (unsigned long)bsp_imu601_rx_total());
    ssd1306_printf(0, 6, "OK: Back");
}

/* ---- Q4: PID 调试 ---- */
static void q4_init(void) { motor_move_to_ex(90.0f, 30.0f); }
static void q4_display(void)
{
    float ang = motor_angle();
    ssd1306_printf(0, 0, "Q4: PID Tune");
    ssd1306_printf(0, 2, "Ang:%3d",  (int)ang);
    ssd1306_printf(0, 4, "Spd:%3d Tgt:%3d",
                   (int)motor_speed(), (int)motor_target_speed());
    ssd1306_printf(0, 6, "OK: Back");
}

/* ---- Q5: 陀螺仪 + 电机联动（示例） ---- */
/* 用 WT901 的 Yaw 角度控制电机：转动传感器 → 电机跟着转 */
static void q5_init(void) {}
static void q5_display(void)
{
    float yaw   = bsp_gyro_get_yaw();
    float wz    = bsp_gyro_get_wz();
    float m_ang = motor_angle();

    /* 简单映射：陀螺仪 Yaw → 电机目标角度 */
    /* Yaw 范围 -180~180，电机 360° 连续，把 Yaw 当目标 */
    motor_move_to_ex(yaw, 120.0f);   /* 120RPM 最大速度 */

    ssd1306_printf(0, 0, "Q5: Gyro->Motor");
    ssd1306_printf(0, 2, "Yaw:%3d Ang:%3d", (int)yaw, (int)m_ang);
    ssd1306_printf(0, 4, "Wz:%3d Spd:%3d",  (int)wz, (int)motor_speed());
    ssd1306_printf(0, 6, "OK: Back");
}

/* ---- 函数表 ---- */
typedef void (*QuestionFunc)(void);
static struct { QuestionFunc init; QuestionFunc display; QuestionFunc exit; } g_questions[] = {
    { q1_init, q1_display, stop_motor },
    { q2_init, q2_display, stop_motor },
    { q3_init, q3_display, noop        },
    { q4_init, q4_display, stop_motor },
    { q5_init, q5_display, stop_motor  },
};

/* ================================================================
 *  任务句柄
 * ================================================================ */
static TaskHandle_t g_ctrl_task;
static TaskHandle_t g_disp_task;

/* 裸机时代的 TIMA0 ISR — FreeRTOS 用 SysTick，清除残留中断 */
extern "C" void TIMA0_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_TICK_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
}

/* ================================================================
 *  控制任务 — 1ms 周期（常驻后台，不随菜单切换而停）
 *
 *  编码器 + 速度计算 + 双陀螺仪数据采集
 * ================================================================ */
static void vControlTask(void *pvParams)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        MT6816_Data enc;
        mt6816_read(&enc);
        motor_measure_tick(&enc);

        bsp_gyro_poll();
        bsp_imu601_poll();

        /* PWM 控制每 5ms 一次 */
        static int tick = 0;
        if (++tick >= 5)
        {
            tick = 0;
            motor_tick(&enc);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

/* ================================================================
 *  显示任务 — 50ms 周期
 *
 *  菜单模式：按钮翻页 + 确认进入题目
 *  题目模式：执行题目的 display()，确认返回菜单
 * ================================================================ */
static void vDisplayTask(void *pvParams)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int cur_q = -1;   /* 当前激活的题目索引（-1=菜单模式） */

    while (1)
    {
        menu_ui_poll(&g_menu_ui);

        if (menu_ui_in_task(&g_menu_ui))
        {
            /* ---- 题目运行模式 ---- */
            int sel = menu_ui_selected(&g_menu_ui);
            if (sel >= 0 && sel < g_menu_count)
            {
                /* 刚确认进入 → 记录题目并调 init */
                cur_q = sel;
                g_questions[cur_q].init();
            }
            /* 每帧调用题目的 display */
            if (cur_q >= 0 && cur_q < g_menu_count)
                g_questions[cur_q].display();
        }
        else
        {
            /* ---- 退回菜单 → 清理 ---- */
            if (cur_q >= 0 && cur_q < g_menu_count)
            {
                g_questions[cur_q].exit();
            }
            cur_q = -1;
            menu_ui_render(&g_menu_ui);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

/* ================================================================
 *  主函数
 * ================================================================ */
int main()
{
    SYSCFG_DL_init();

    ssd1306_init();
    ssd1306_printf(0, 0, "  FreeRTOS OK");
    ssd1306_printf(0, 2, " Booting...");

    mt6816_init();
    motor_init();
    bsp_gyro_init();
    bsp_imu601_init();

    bsp_uart_printf("=== FreeRTOS Motor + Dual Gyro + OLED Menu ===\r\n");

    /* 初始化菜单 */
    menu_init(&g_menu_state, g_menu, g_menu_count);
    menu_ui_init(&g_menu_ui, &g_menu_state);

    /* 任务创建：Ctrl 3 > Disp 2 > Idle 0 */
    xTaskCreate(vControlTask, "Ctrl", 256, NULL, 3, &g_ctrl_task);
    xTaskCreate(vDisplayTask, "Disp", 512, NULL, 2, &g_disp_task);

    vTaskStartScheduler();

    /* 不会运行到这里 */
    while (1) {}
}
