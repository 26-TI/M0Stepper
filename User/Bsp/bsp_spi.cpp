/**
 * @file bsp_spi.cpp
 * @brief BSP SPI — 单字节轮询 + 非阻塞 DMA (8/16 位自适应)
 */

#include "bsp_spi.h"
#include "ti_msp_dl_config.h"

static const uint32_t TIMEOUT = 65535;

void BspSpi::init(const Config &cfg)
{
    dataBits_ = cfg.dataBits;
}

/* ================================================================
 *  单字节轮询
 * ================================================================ */

uint16_t BspSpi::transfer(uint16_t tx)
{
    uint32_t to;

    to = TIMEOUT;
    while (DL_SPI_isTXFIFOFull(SPI_1_INST) && --to) { }
    if (to == 0) return 0;

    if (dataBits_ >= 16)
        DL_SPI_transmitData16(SPI_1_INST, (uint16_t)tx);
    else
        DL_SPI_transmitData8(SPI_1_INST, (uint8_t)tx);

    to = TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_1_INST) && --to) { }
    if (to == 0) return 0;

    if (dataBits_ >= 16)
        return DL_SPI_receiveData16(SPI_1_INST);
    else
        return DL_SPI_receiveData8(SPI_1_INST);
}

/* ================================================================
 *  非阻塞 DMA
 * ================================================================ */
#if defined(DMA_CH0_CHAN_ID) && defined(DMA_CH1_CHAN_ID)

#include <ti/driverlib/dl_dma.h>

static BspSpi            *g_dmaOwner = nullptr;
static BspSpi::DmaCallback g_dmaCb    = nullptr;

bool BspSpi::startDma(const void *tx, void *rx, uint16_t len, DmaCallback cb)
{
    if (!dmaDone_) return false;   /* 上一次还没完 */

    dmaCb_    = cb;
    dmaDone_  = false;
    g_dmaOwner = this;
    g_dmaCb    = cb;

    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)tx);
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)&SPI_1_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, len);

    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&SPI_1_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)rx);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, len);

    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0);

    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);

    return true;
}

bool BspSpi::isDmaBusy() const { return !dmaDone_; }

/* DMA ISR: TX 完成 → 等 RX → 调回调 */
static void dmaIsrHandler()
{
    if (DL_DMA_getEnabledInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0))
    {
        DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
        DL_DMA_disableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0);

        while (DL_DMA_isChannelEnabled(DMA, DMA_CH0_CHAN_ID)) { }

        if (g_dmaOwner) g_dmaOwner->dmaDone_ = true;

        BspSpi::DmaCallback cb = g_dmaCb;
        g_dmaCb = nullptr;
        if (cb) cb();
    }
}

extern "C" void DMA_IRQHandler(void) { dmaIsrHandler(); }

#else

bool BspSpi::startDma(const void *, void *, uint16_t, DmaCallback) { return false; }
bool BspSpi::isDmaBusy() const { return false; }

#endif
