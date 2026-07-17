/**
 * @file bsp_spi.cpp
 * @brief BSP SPI — MSPM0 SPI1 实现 (Mode 3, 8MHz)
 *
 * 引脚：PB7=POCI, PB8=PICO, PB9=SCLK, PB6=CS（软件控制见 bsp_gpio）
 */

#include "bsp_spi.h"
#include "ti_msp_dl_config.h"

void bsp_spi_init(void)
{
    /* SPI1 已在 SYSCFG_DL_init 中配好，此处无需操作 */
}

uint8_t bsp_spi_transfer(uint8_t tx)
{
    /* 等 TX FIFO 不满 */
    while (DL_SPI_isTXFIFOFull(SPI_1_INST)) { }

    DL_SPI_transmitData8(SPI_1_INST, tx);

    /* 等 RX FIFO 不空（有数据可读） */
    while (DL_SPI_isRXFIFOEmpty(SPI_1_INST)) { }

    return DL_SPI_receiveData8(SPI_1_INST);
}
