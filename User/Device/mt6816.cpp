/**
 * @file mt6816.cpp
 * @brief MT6816 磁编码器驱动 — SPI 角度读取（BSP 抽象层）
 *
 * 通信协议（SPI Mode 3, CPOL=1, CPHA=1）：
 *   1. CS 拉低 → 发 0x83 + dummy → 收到角度高 8 位 [13:6]
 *   2. CS 拉高 → CS 拉低 → 发 0x84 + dummy → 收到角度低 6 位 [5:0]
 *   3. 角度拼接：raw = ((high << 8) | low) >> 2
 *   4. 零值毛刺过滤：跳 0 且 delta > 90° → 丢弃
 */

#include "mt6816.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"

#define MT6816_REG_HIGH   0x83   ///< 读 0x03：角度高 8 位
#define MT6816_REG_LOW    0x84   ///< 读 0x04：角度低 6 位 + 状态

void mt6816_init(void)
{
    bsp_cs_high();
}

void mt6816_read(MT6816_Data *data)
{
    uint8_t data_high, data_low;

    /* ---- 读角度高 8 位 ---- */
    bsp_cs_low();
    bsp_delay_us(2);
    bsp_spi_transfer(MT6816_REG_HIGH);      // 发读命令
    data_high = bsp_spi_transfer(0x00);     // dummy → 收到高 8 位
    bsp_cs_high();
    bsp_delay_us(2);

    /* ---- 读角度低 6 位 ---- */
    bsp_cs_low();
    bsp_delay_us(2);
    bsp_spi_transfer(MT6816_REG_LOW);       // 发读命令
    data_low = bsp_spi_transfer(0x00);      // dummy → 收到低 6 位 + 状态
    bsp_cs_high();

    /* ---- 保存原始数据 ---- */
    data->raw_h = data_high;
    data->raw_l = data_low;
    data->mag   = (data_low & 0x02) >> 1;

    /* ---- 磁铁正常才更新角度 ---- */
    if (data->mag == 0)
    {
        int new_raw = ((data_high << 8) | data_low) >> 2;

        /* 毛刺检测：跳到 0 且 delta > 90°(4096 raw)，不更新 */
        int delta = new_raw - data->raw_angle;
        if (delta < 0) delta = -delta;
        if (delta > 8192) delta = 16384 - delta;

        if (!(new_raw == 0 && delta > 4096))
        {
            data->raw_angle = new_raw;
            data->angle     = data->raw_angle * 360.0f / 16384.0f;
        }
    }
}
