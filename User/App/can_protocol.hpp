/**
 * @file can_protocol.hpp
 * @brief CAN 通信协议 — 指令解析 + 状态上报
 */

#ifndef __CAN_PROTOCOL_HPP__
#define __CAN_PROTOCOL_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_ID_CMD   0x100   ///< 主机 → MSPM0 指令
#define CAN_ID_STAT  0x200   ///< MSPM0 → 主机 状态

/// 指令类型
enum {
    CMD_NONE      = 0x00,
    CMD_SET_ANGLE = 0x01,   ///< 角度控制 (×100 deg, max_rpm)
    CMD_STOP      = 0x02,   ///< 立即停止
    CMD_OPEN_LOOP = 0x03,   ///< 开环调速 (×10 RPM)
};

/// 状态码
enum {
    STAT_IDLE   = 0,       ///< 空闲
    STAT_MOVING = 1,       ///< 运动中
    STAT_DONE   = 2,       ///< 到位
    STAT_ERROR  = 3,       ///< 故障
};

void can_proto_init(void);          ///< 初始化
void can_proto_tick(void);          ///< 每 1ms 调用

#ifdef __cplusplus
}
#endif

#endif
