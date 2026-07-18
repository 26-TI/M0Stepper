/**
 * @file bsp_gpio.cpp
 * @brief BSP GPIO — MSPM0 实现
 *
 * 引脚映射（SysConfig 自动生成宏）：
 *   CS  → PB6  (SPI_CS_SPI_CS0_PIN / SPI_CS_PORT)
 *   DIR → PA27 (STP_pins_Dir_PIN)
 *   LED → PA14 (LED_LED22_PIN / LED_PORT)
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

/* ---- 按键（SysConfig 上拉 → 按下=低电平） ---- */
int bsp_btn_up(void) { return DL_GPIO_readPins(Keys_PORT, Keys_Key1_up_PIN) == 0; }
int bsp_btn_dn(void) { return DL_GPIO_readPins(Keys_PORT, Keys_Key2_dn_PIN) == 0; }
int bsp_btn_ok(void) { return DL_GPIO_readPins(Keys_PORT, Keys_Key3_ok_PIN) == 0; }
