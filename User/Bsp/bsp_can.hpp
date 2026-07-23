/**
 * @file bsp_can.hpp
 * @brief CAN BSP 层 — 非阻塞发送 + 环形接收缓冲 + ISR
 *
 * PA12=CAN_TX, PA13=CAN_RX, 1Mbps, MCAN0
 * 全局单例 bsp_can，ISR 通过 bsp_can.isrHandler() 驱动接收
 */

#ifndef __BSP_CAN_HPP__
#define __BSP_CAN_HPP__

#include "ti_msp_dl_config.h"
#include <stdint.h>

class BspCan
{
public:
    enum Mode { NORMAL = 0, LOOPBACK = 1 };

    struct Status {
        uint32_t txErrCnt;
        uint32_t rxErrCnt;
        uint32_t activity;
        uint32_t lastErrCode;
        bool     busOff;
    };

    void init(Mode mode = NORMAL);
    void enableIrq();
    void disableIrq();

    bool send(uint32_t id, const uint8_t *data, uint8_t len);
    void recvPoll();                                    ///< 轮询模式收帧
    bool recvRead(uint32_t *id, uint8_t *data, uint8_t *len);

    void setLoopback(bool enable);
    void getStatus(Status *st);

    /* ISR 回调，由 CANFD0_IRQHandler 调用 */
    void isrHandler();

private:
    static constexpr uint8_t RX_BUF_SIZE = 16;

    volatile uint32_t rxId_[RX_BUF_SIZE];
    volatile uint8_t  rxData_[RX_BUF_SIZE][8];
    volatile uint8_t  rxLen_[RX_BUF_SIZE];
    volatile uint8_t  rxHead_ = 0;
    volatile uint8_t  rxTail_ = 0;
    volatile uint8_t  rxCnt_  = 0;
    bool              loopback_ = false;

    void writeUnlock();
    void writeLock();
};

extern BspCan bsp_can;

/* ---- 兼容旧全局函数 API ---- */
#define CAN_MODE_NORMAL   0
#define CAN_MODE_LOOPBACK  1
inline void CAN_Init(int m)     { bsp_can.init((BspCan::Mode)m); }
inline bool CAN_Send(uint32_t id, const uint8_t *d, uint8_t l) { return bsp_can.send(id, d, l); }
inline void CAN_RecvPoll(void)   { bsp_can.recvPoll(); }
inline bool CAN_RecvRead(uint32_t *i, uint8_t *d, uint8_t *l) { return bsp_can.recvRead(i, d, l); }
inline void CAN_EnableIrq(void)  { bsp_can.enableIrq(); }
inline void CAN_DisableIrq(void) { bsp_can.disableIrq(); }

#endif
