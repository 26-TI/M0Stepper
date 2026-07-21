/**
 * @file main.cpp
 * @brief 步进电机 FreeRTOS — Ctrl(1ms) + Demo(10ms) + CAN
 */

#define MOTOR_ID  1

#include "main.hpp"
#include "motor_control.hpp"
#include "bsp_gpio.h"
#include "bsp_can.hpp"
#include "can_protocol.hpp"

#include "FreeRTOS.h"
#include "task.h"

#define F1D(v) (int)(v), (int)(((v)-(int)(v))*10.0f+0.5f)%10

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
        motor_measure_tick(&enc);

        static int tick = 0;
        if (++tick >= 5) { tick = 0; motor_tick(&enc); }

        can_proto_tick();

        static int pt = 0;
        if (++pt >= 100)
        {
            pt = 0;
            float rpm = motor_speed();
            bsp_led_toggle();
            bsp_uart_printf("E:%d.%ddeg %d.%drpm\r\n",
                            F1D(enc.angle), F1D(rpm));
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

static void vDemoTask(void *pvParams)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t tick = 0;

    while (1)
    {
        switch (tick / 100)
        {
            case 0:  motor_set_speed(120);            break;
            case 5:  motor_set_speed(0);              break;
            case 6:  motor_move_to_ex(90, 60);       break;
            case 11: motor_move_to_ex(180, 60);      break;
            case 16: motor_set_speed(-120);           break;
            case 20: tick = 0; continue;
        }
        tick++;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

int main()
{
    SYSCFG_DL_init();
    mt6816_init();
    motor_init();
    can_proto_init();

    bsp_uart_printf("\r\n=== FreeRTOS Motor + CAN ===\r\n");

    xTaskCreate(vCtrlTask, "Ctrl", 512, NULL, 3, NULL);
    //xTaskCreate(vDemoTask, "Demo", 256, NULL, 2, NULL);

    vTaskStartScheduler();
    while (1) {}
}
