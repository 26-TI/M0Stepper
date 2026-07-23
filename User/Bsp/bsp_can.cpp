/**
 * @file bsp_can.cpp
 * @brief CAN BSP 层实现 — 非阻塞发送 + 环形接收缓冲 + ISR
 */

#include "bsp_can.hpp"
#include <string.h>

/* ---- 全局实例 ---- */
BspCan bsp_can;

/* ---- ISR ---- */
extern "C" void MCAN0_INST_IRQHandler(void)
{
    bsp_can.isrHandler();
}

/* ================================================================
 *  辅助
 * ================================================================ */

void BspCan::writeUnlock()
{
    MCAN0_INST->MCANSS.MCAN.MCAN_CCCR |= (MCAN_CCCR_CCE_MASK | MCAN_CCCR_INIT_MASK);
    while (0U == (MCAN0_INST->MCANSS.MCAN.MCAN_CCCR & MCAN_CCCR_INIT_MASK));
}

void BspCan::writeLock()
{
    MCAN0_INST->MCANSS.MCAN.MCAN_CCCR &= ~MCAN_CCCR_CCE_MASK;
    MCAN0_INST->MCANSS.MCAN.MCAN_CCCR &= ~MCAN_CCCR_INIT_MASK;
    while (0U != (MCAN0_INST->MCANSS.MCAN.MCAN_CCCR & MCAN_CCCR_INIT_MASK));
}

/* ================================================================
 *  API
 * ================================================================ */

void BspCan::init(Mode mode)
{
    /* 全局滤波器：非匹配帧进 FIFO0 */
    DL_MCAN_ConfigParams cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.filterConfig.anfs = 0;
    cfg.filterConfig.anfe = 0;
    DL_MCAN_config(MCAN0_INST, &cfg);

    rxHead_ = 0;
    rxTail_ = 0;
    rxCnt_  = 0;

    setLoopback(mode == LOOPBACK);
}

bool BspCan::send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (DL_MCAN_getTxBufReqPend(MCAN0_INST) & 0x01U)
        return false;

    DL_MCAN_TxBufElement tx;
    memset(&tx, 0, sizeof(tx));
    tx.id  = (id & 0x7FFU) << 18U;
    tx.dlc = (len > 8) ? 8 : (uint8_t)len;
    if (len > 0 && data) memcpy(tx.data, data, tx.dlc);

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0U, &tx);
    DL_MCAN_TXBufAddReq(MCAN0_INST, 0U);
    return true;
}

void BspCan::recvPoll()
{
    DL_MCAN_RxFIFOStatus st;
    st.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);

    while (st.fillLvl > 0)
    {
        DL_MCAN_RxBufElement rx;
        DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0,
                           DL_MCAN_RX_FIFO_NUM_0, &rx);
        DL_MCAN_writeRxFIFOAck(MCAN0_INST, DL_MCAN_RX_FIFO_NUM_0, st.getIdx);

        if (rxCnt_ >= RX_BUF_SIZE) break;
        uint8_t idx = rxHead_;
        rxId_[idx]  = (rx.id >> 18U) & 0x7FFU;
        rxLen_[idx] = (rx.dlc > 8) ? 8 : (uint8_t)rx.dlc;
        memcpy((void *)rxData_[idx], rx.data, rxLen_[idx]);
        rxHead_ = (idx + 1) % RX_BUF_SIZE;
        rxCnt_++;

        st.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);
    }
}

bool BspCan::recvRead(uint32_t *id, uint8_t *data, uint8_t *len)
{
    if (rxCnt_ == 0) return false;

    uint8_t idx = rxTail_;
    *id  = rxId_[idx];
    *len = rxLen_[idx];
    memcpy(data, (const void *)rxData_[idx], *len);
    rxTail_ = (idx + 1) % RX_BUF_SIZE;
    rxCnt_--;
    return true;
}

void BspCan::getStatus(Status *st)
{
    DL_MCAN_ErrCntStatus ec;
    DL_MCAN_getErrCounters(MCAN0_INST, &ec);

    DL_MCAN_ProtocolStatus ps;
    DL_MCAN_getProtocolStatus(MCAN0_INST, &ps);

    st->txErrCnt    = ec.transErrLogCnt;
    st->rxErrCnt    = ec.recErrCnt;
    st->activity    = ps.act;
    st->lastErrCode = ps.lastErrCode;
    st->busOff      = (ps.busOffStatus != 0);
}

void BspCan::setLoopback(bool enable)
{
    writeUnlock();
    DL_MCAN_lpbkModeEnable(MCAN0_INST, DL_MCAN_LPBK_MODE_INTERNAL, enable);
    loopback_ = enable;
    writeLock();
}

void BspCan::enableIrq()
{
    DL_MCAN_enableIntr(MCAN0_INST, DL_MCAN_INTR_MASK_ALL, 0U);
    DL_MCAN_enableIntr(MCAN0_INST,
        DL_MCAN_INTERRUPT_RF0N | DL_MCAN_INTERRUPT_RF0L, 1U);
    DL_MCAN_clearInterruptStatus(MCAN0_INST, DL_MCAN_MSP_INTERRUPT_LINE0);
    __DSB();
    NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);
}

void BspCan::disableIrq()
{
    NVIC_DisableIRQ(MCAN0_INST_INT_IRQN);
}

/* ================================================================
 *  ISR: FIFO → 环形缓冲
 * ================================================================ */

void BspCan::isrHandler()
{
    uint32_t status = DL_MCAN_getIntrStatus(MCAN0_INST);
    DL_MCAN_clearIntrStatus(MCAN0_INST, status, DL_MCAN_INTR_SRC_MCAN_LINE_0);

    if (status & MCAN_IR_RF0N_MASK)
    {
        DL_MCAN_RxFIFOStatus st;
        st.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);

        while (st.fillLvl > 0)
        {
            DL_MCAN_RxBufElement rx;
            DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0,
                               DL_MCAN_RX_FIFO_NUM_0, &rx);
            DL_MCAN_writeRxFIFOAck(MCAN0_INST, DL_MCAN_RX_FIFO_NUM_0,
                                   st.getIdx);

            if (rxCnt_ >= RX_BUF_SIZE) break;
            uint8_t idx = rxHead_;
            rxId_[idx]  = (rx.id >> 18U) & 0x7FFU;
            rxLen_[idx] = (rx.dlc > 8) ? 8 : (uint8_t)rx.dlc;
            memcpy((void *)rxData_[idx], rx.data, rxLen_[idx]);
            rxHead_ = (idx + 1) % RX_BUF_SIZE;
            rxCnt_++;

            st.num = DL_MCAN_RX_FIFO_NUM_0;
            DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);
        }
    }
}
