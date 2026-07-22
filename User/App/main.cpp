/**
 * @file main.cpp
 * @brief 步进电机 FreeRTOS — Ctrl(1ms) + CAN
 */

#define MOTOR_ID  1

#include "main.hpp"
#include "motor_control.hpp"
#include "can_protocol.hpp"
#include "bsp_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#define F1D(v) (int)(v), (int)(((v)-(int)(v))*10.0f+0.5f)%10
#define F2D(v) (int)(v), (int)(((v)-(int)(v))*100.0f+0.5f)%100

/* ---- 全局对象 ---- */
static MotorControl g_motor;
static CanProtocol  g_can(g_motor, MOTOR_ID);

extern "C" void TIMA0_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_TICK_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
}

static void vCtrlTask(void *pvParams)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        MT6816_Data enc;
        mt6816_read(&enc);
        g_motor.measureTick(&enc);

        static int tick = 0;
        if (++tick >= 5) { tick = 0; g_motor.controlTick(&enc); }

        g_can.tick();

        static int pt = 0;
        if (++pt >= 100)
        {
            pt = 0;
            bsp_led_toggle();
            bsp_uart_printf("Tgt:%d.%d Act:%d.%d Ang:%d.%02d\r\n",
                            F1D(g_motor.targetSpeed()), F1D(g_motor.speed()),
                            F2D(g_motor.angle()));
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

int main()
{
    SYSCFG_DL_init();
    mt6816_init();
    g_motor.init();
    g_can.init();

    bsp_uart_printf("\r\n=== FreeRTOS Motor + CAN ===\r\n");

    xTaskCreate(vCtrlTask, "Ctrl", 512, NULL, 3, NULL);

    vTaskStartScheduler();
    while (1) {}
}
