/**
 * @file bsp_i2c.cpp
 * @brief BSP I2C — MSPM0 I2C0 实现 (500kHz Fast)
 *
 * 引脚：PA0=SDA, PA1=SCL
 * 注意：TX FIFO 8 字节深，一次不要发超过 8 字节
 */

#include "bsp_i2c.h"
#include "ti_msp_dl_config.h"

void bsp_i2c_send(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, len);

    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) { }

    DL_I2C_startControllerTransfer(I2C_0_INST, addr,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, len);
}
