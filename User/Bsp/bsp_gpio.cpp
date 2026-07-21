/**
 * @file bsp_gpio.cpp
 * @brief BSP GPIO — MSPM0 实现
 *
 * 引脚映射（SysConfig 自动生成宏）：
 *   CS  → PB6  (SPI_CS_SPI_CS0_PIN / SPI_CS_PORT)
 *   DIR → PA27 (STP_pins_Dir_PIN)
 *   LED → PA0  (LED_LED22_PIN / LED_PORT)
 */

#include "bsp_gpio.h"
#include "ti_msp_dl_config.h"

void bsp_cs_high(void)
{
    DL_GPIO_setPins(SPI_CS_PORT, SPI_CS_SPI_CS0_PIN);
}

void bsp_cs_low(void)
{
    DL_GPIO_clearPins(SPI_CS_PORT, SPI_CS_SPI_CS0_PIN);
}

void bsp_dir_high(void)
{
    DL_GPIO_setPins(GPIOA, STP_pins_Dir_PIN);
}

void bsp_dir_low(void)
{
    DL_GPIO_clearPins(GPIOA, STP_pins_Dir_PIN);
}

void bsp_led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_LED22_PIN);
}

void bsp_delay_us(uint32_t us)
{
    /* 80MHz 下每 us ~80 个 NOP，粗略估算 */
    for (volatile uint32_t i = 0; i < us * 60; i++)
        __asm("nop");
}
