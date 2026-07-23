/**
 * @file bsp_spi.h
 * @brief BSP SPI — 硬件 CS + 轮询/DMA 传输 (MSPM0 SPI1, Mode 3, 8MHz)
 */

#ifndef __BSP_SPI_HPP__
#define __BSP_SPI_HPP__

#include <stdint.h>

class BspSpi
{
public:
    typedef void (*DmaCallback)(void);    ///< DMA 完成回调

    struct Config
    {
        uint8_t dataBits = 8;   ///< 数据位宽: 8 或 16
    };

    void     init(const Config &cfg = {8});

    /* ---- 单字节轮询 (发寄存器地址之类, ~2μs) ---- */
    uint16_t transfer(uint16_t tx);

    /* ---- 非阻塞 DMA (日常用这个) ---- */
    bool     startDma(const void *tx, void *rx, uint16_t len,
                      DmaCallback cb = nullptr); ///< 立即返回, 完成时调 cb
    bool     isDmaBusy() const;                  ///< DMA 正在跑?

    /* DMA ISR 需要直接写这些 */
    volatile bool dmaDone_  = false;
    DmaCallback   dmaCb_    = nullptr;

private:
    uint8_t dataBits_ = 8;
};

#endif
