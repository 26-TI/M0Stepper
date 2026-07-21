/**
 * @file can_protocol.hpp
 * @brief 主控端 CAN 电机控制库 — 初始化 + 收发 + 状态缓存，一站式 API
 *
 * 用法:
 *   can_motor_init();                        // 初始化
 *   can_motor_tick();                        // 每 1ms 调用
 *   can_motor_set_speed(1, 60);              // 电机1 正转 60 RPM
 *   can_motor_move_to(1, 90, 45);            // 电机1 转到 90°, 限速 45
 *   can_motor_get_angle(1);                  // 读电机1 角度
 *   can_motor_is_done(1);                    // 电机1 到位?
 *
 * 协议:
 *   命令帧 0x100+N: 8 字节 [0]cmd [1-2]p1 [3-4]p2 [5-6]p3 [7]XOR
 *   状态帧 0x200+N: 8 字节 [0]stat [1-2]ang×100 [3-4]rpm×10 [5-6]turns [7]XOR
 */

#ifndef __CAN_PROTOCOL_HPP__
#define __CAN_PROTOCOL_HPP__

#include <stdint.h>
#include <stdbool.h>
#include "m0stepper_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  生命周期
 * ================================================================ */

void can_motor_init(void);          ///< 初始化 CAN + 绑定发送函数
void can_motor_tick(void);          ///< 每 1ms 调用 — 收状态帧 + 更新缓存

/* ================================================================
 *  电机控制命令（CAN 发送，不阻塞）
 * ================================================================ */

void can_motor_set_speed(uint8_t id, float rpm);
void can_motor_move_to(uint8_t id, float angle_deg, float max_rpm);
void can_motor_stop(uint8_t id);
void can_motor_enable(uint8_t id, bool on);
void can_motor_query(uint8_t id, uint8_t sub);   ///< sub: 0=全 1=角 2=速 3=圈

/* ================================================================
 *  状态读取（本地缓存，CAN 收到后自动更新）
 * ================================================================ */

bool   can_motor_has_status(uint8_t id);              ///< 是否收到过状态
float  can_motor_get_angle(uint8_t id);               ///< 当前角度 °
float  can_motor_get_speed(uint8_t id);               ///< 当前转速 RPM
int    can_motor_get_state(uint8_t id);               ///< 0=空闲 1=运动 2=到位 3=故障
int    can_motor_get_turns(uint8_t id);               ///< 累计圈数
bool   can_motor_is_done(uint8_t id);                 ///< 是否到位
bool   can_motor_get_full_status(uint8_t id, M0Stepper_Status *st);  ///< 获取完整状态

#ifdef __cplusplus
}
#endif

#endif
