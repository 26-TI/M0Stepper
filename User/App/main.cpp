/**
 * @file main.cpp
 * @brief 步进电机 FreeRTOS 多任务分离
 *
 * 任务架构:
 *   vMotorTask   (prio 4, 1ms):  编码器读取 + 圈数追踪 + GOTO/速度控制
 *   vCanTask     (prio 3, 1ms):  CAN 收指令 + 50ms 状态上报
 *   vMonitorTask (prio 1, 100ms): LED 闪烁 + 串口打印
 */

#define MOTOR_ID  1

#include "main.hpp"
#include "motor_control.hpp"
#include "can_protocol.hpp"
#include "bsp_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define F1D(v) (int)(v), (int)(((v)>0?((v)-(int)(v)):-((v)-(int)(v)))*10.0f+0.5f)
#define F2D(v) (int)(v), (int)(((v)>0?((v)-(int)(v)):-((v)-(int)(v)))*100.0f+0.5f)

/* ---- 全局对象 ---- */
static MotorControl g_motor;
static CanProtocol  g_can(g_motor, MOTOR_ID);
static SemaphoreHandle_t g_mutex;  // 保护 g_motor 的互斥锁

extern "C" void TIMA0_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_TICK_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
}

/* ================================================================
 *  电机控制任务 (prio 4, 1ms)
 *   编码器读取 + 测速 + 圈数追踪 + GOTO/速度/定时控制
 * ================================================================ */
static void vMotorTask(void *pvParams)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static int ctrlTick = 0;

    while (1)
    {
        MT6816_Data enc;
        mt6816_read(&enc);

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_motor.measureTick(&enc);
        if (++ctrlTick >= 5) {
            ctrlTick = 0;
            g_motor.controlTick(&enc);
        }
        xSemaphoreGive(g_mutex);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

/* ================================================================
 *  CAN 通信任务 (prio 3, 1ms)
 *   收指令 → 解析 → 发给电机 / 50ms 上报状态
 * ================================================================ */
static void vCanTask(void *pvParams)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_can.tick();
        xSemaphoreGive(g_mutex);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

/* ================================================================
 *  监控任务 (prio 1, 100ms)
 *   LED 闪烁 + 串口状态打印
 * ================================================================ */
static void vMonitorTask(void *pvParams)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        float target = g_motor.targetSpeed();
        float actual = g_motor.speed();
        float angle  = g_motor.angle();
        xSemaphoreGive(g_mutex);

        bsp_led_toggle();
        bsp_uart_printf("Tgt:%d.%d Act:%d.%d Ang:%d.%02d\r\n",
                        F1D(target), F1D(actual), F2D(angle));

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/* ================================================================
 *  入口
 * ================================================================ */
int main()
{
    SYSCFG_DL_init();
    mt6816_init();
    g_motor.init();
    g_can.init();

    g_mutex = xSemaphoreCreateMutex();

    bsp_uart_printf("\r\n=== FreeRTOS Motor + CAN (multi-task) ===\r\n");

    xTaskCreate(vMotorTask,   "Motor",   512, NULL, 4, NULL);
    xTaskCreate(vCanTask,     "CAN",     512, NULL, 3, NULL);
    xTaskCreate(vMonitorTask, "Monitor", 256, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) {}
}
