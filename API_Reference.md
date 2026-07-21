# m0stepper — 步进电机控制平台 API 参考手册

> 工程：`d:/Ti/test/m0stepper`  
> 芯片：MSPM0G3507 (Cortex-M0+, 80MHz, LQFP-48)  
> 架构：Bsp → Device → Module → App 四层

---

## 目录

- [App 层 — 应用调度](#app-层)
  - [motor_control — 电机统一控制](#motor_control)
  - [can_protocol — CAN 多设备协议](#can_protocol)
- [Module 层 — 算法模块](#module-层)
  - [pid — 通用 PID 控制器](#pid)
  - [speed_measure — 编码器测速](#speed_measure)
- [Device 层 — 器件驱动](#device-层)
  - [stepper_motor — 步进电机 STEP+DIR](#stepper_motor)
  - [mt6816 — 磁编码器](#mt6816)
- [Bsp 层 — MCU 硬件抽象](#bsp-层)
  - [bsp_pwm — STEP 脉冲](#bsp_pwm)
  - [bsp_spi — 编码器 SPI](#bsp_spi)
  - [bsp_can — CAN 总线](#bsp_can)
  - [bsp_gpio — 通用 GPIO](#bsp_gpio)
  - [bsp_uart — 调试串口](#bsp_uart)
- [附录：引脚分配表](#附录引脚分配表)

---

## App 层

### motor_control

步进电机统一控制——速度开环 + 位置闭环 (GOTO)。

**依赖**：`stepper_motor.h` `mt6816.h` `speed_measure.h`

```c
#include "motor_control.hpp"

// === 初始化 ===
void motor_init(void);

// === 速度控制（开环） ===
void motor_set_speed(float rpm);   // 正转 >0，反转 <0
void motor_stop(void);             // 停转

// === 位置控制（编码器闭环 P） ===
void motor_move_to(float angle_deg);                       // 转到目标角，限速 45 RPM
void motor_move_to_ex(float angle_deg, float max_rpm);     // 转到目标角，自定义限速
int  motor_move_done(void);                                // 到位返回 1

// === 数据读取 ===
float motor_speed(void);           // 实测 RPM
float motor_angle(void);           // 编码器角度 (°)
float motor_target_speed(void);    // 当前目标 RPM
int   motor_is_moving(void);       // 运动中返回 1

// === 1ms 调度 ===
void motor_tick(MT6816_Data *enc);          // 控制逻辑，5ms 一次
void motor_measure_tick(MT6816_Data *enc);  // 测速 + 角度记录，1ms 一次
```

---

### can_protocol

CAN 总线通信协议（250kbps CAN 2.0 标准帧），支持多设备。

**硬件**：PA12(CAN_TX) / PA13(CAN_RX)，需外接 CAN 收发器

#### CAN ID 分配

| 电机 | 命令帧（主机→MSPM0） | 状态帧（MSPM0→主机） |
|------|---------------------|---------------------|
| 1 | `0x101` | `0x201` |
| 2 | `0x102` | `0x202` |
| N | `0x100 + N` | `0x200 + N` |

固件烧录前改 `main.cpp` 顶部 `#define MOTOR_ID N`。

**硬件滤波器**：MCAN0 只收本机命令 ID，其他自动丢弃。

#### 命令帧格式（`0x10N`，主机→MSPM0）

| 字节 | 含义 |
|------|------|
| Data[0] | 指令类型 |
| Data[1-2] | 参数1（int16, 大端） |
| Data[3-4] | 参数2（int16, 大端） |

| 指令 | 码 | 参数1 | 参数2 |
|------|----|------|------|
| CMD_SET_ANGLE | 0x01 | 目标角度×100 | 最大 RPM |
| CMD_STOP | 0x02 | — | — |
| CMD_OPEN_LOOP | 0x03 | 目标转速×10 RPM | — |
| CMD_QUERY | 0x04 | — | — |

示例：电机 1 转到 90°，限速 60 RPM
```
CAN ID: 0x101  Data: [0x01, 0x23, 0x28, 0x00, 0x3C]
                   cmd   angle=90×100=9000=0x2328  max=60
```

#### 状态帧格式（`0x20N`，MSPM0→主机，每 50ms）

| 字节 | 含义 |
|------|------|
| Data[0] | 状态码 |
| Data[1-2] | 当前角度×100（int16, 大端） |
| Data[3-4] | 当前转速×10（int16, 大端） |
| Data[5] | 错误码 |

| 状态码 | 含义 |
|--------|------|
| 0 | 空闲 |
| 1 | 运动中 |
| 2 | 到位 |
| 3 | 故障 |

```c
#include "can_protocol.hpp"

void can_proto_init(void);    // CAN 初始化 + 配置滤波器
void can_proto_tick(void);    // 1ms 调用：收指令执行 + 50ms 上报状态
```

---

## Module 层

### pid

通用位置式 PID 控制器，带积分抗饱和 (back-calculation) 和输出限幅。纯算法，零硬件依赖。

```c
#include "pid.h"

typedef struct {
    float kp, ki, kd;
    float integral, prev_error;
    float integral_limit, output_limit;
} PID;

void  pid_init(PID *pid, float kp, float ki, float kd,
               float integral_limit, float output_limit);
float pid_update(PID *pid, float target, float actual, float dt);
```

---

### speed_measure

编码器角度差分测速，低通滤波 (α=0.5)。

```c
#include "speed_measure.h"

typedef struct {
    float actual_rpm;
    float last_angle;
    float speed_filtered;
    int   first_read;
} SpeedMeasure;

void speed_measure_init(SpeedMeasure *sm);
int  speed_measure_update(SpeedMeasure *sm, MT6816_Data *enc);  // 有新数据返回 1
```

---

## Device 层

### stepper_motor

步进电机 STEP+DIR 驱动。

**引脚**：STEP=PA26(TIMG7 CCP0), DIR=PA27, EN=PA25（低电平使能）

```c
#include "stepper_motor.h"

#define STEPS_PER_REV 6400   // 200 步/圈 × 32 细分

void stepper_init(void);               // DIR=正转, EN=拉低使能, 停脉冲
void stepper_set_speed(float rpm);     // >0 正转, <0 反转, |rpm|<0.5 停
```

---

### mt6816

MT6816 14 位磁编码器，SPI Mode 3 (CPOL=1, CPHA=1)，8MHz。

```c
#include "mt6816.h"

typedef struct {
    uint8_t mag;        // 0=正常, 1=异常
    uint8_t raw_h, raw_l;
    int     raw_angle;   // 0~16383
    float   angle;       // 0°~360°
} MT6816_Data;

void mt6816_init(void);                  // 拉高 CS
void mt6816_read(MT6816_Data *data);     // 读一次角度（含毛刺过滤）
```

**毛刺过滤**：突然跳到 0 且跳幅 >90°(4096 raw) 时拒更新。

---

## Bsp 层

MCU 硬件抽象——**换芯片唯一需要重写的层**。

### bsp_pwm

```c
#include "bsp_pwm.h"

void bsp_step_pwm_init(void);              // 初始化 TIMG7（1MHz），停计数器
void bsp_step_pwm_set_freq(uint32_t hz);   // 0=停，其他=设频率并启动
```

---

### bsp_spi

```c
#include "bsp_spi.h"

void     bsp_spi_init(void);                  // SPI1 已在 SYSCFG_DL_init 中配置
uint8_t  bsp_spi_transfer(uint8_t tx);        // 全双工单字节
```

---

### bsp_can

```c
#include "bsp_can.h"

void bsp_can_init(void);
void bsp_can_set_filter(uint32_t rx_id);    // 只接收指定标准帧 ID（精确匹配）
bool bsp_can_send(uint32_t id, uint8_t *data, uint8_t len);   // 标准帧，≤8 字节
bool bsp_can_recv(uint32_t *id, uint8_t *data, uint8_t *len); // 非阻塞
bool bsp_can_has_msg(void);                                     // 有新消息返回 true
```

---

### bsp_gpio

```c
#include "bsp_gpio.h"

void bsp_cs_high(void);       // PB6 → 高（编码器 CS）
void bsp_cs_low(void);        // PB6 → 低
void bsp_dir_high(void);      // PA27 → 高（电机正转）
void bsp_dir_low(void);       // PA27 → 低（电机反转）
void bsp_led_toggle(void);    // PA0 LED 翻转
void bsp_delay_us(uint32_t us);
```

---

### bsp_uart

```c
#include "bsp_uart.h"

int bsp_uart_printf(const char *fmt, ...);    // 阻塞式 printf
// 注意：newlib-nano 不支持 %f，用 F1D(v) 宏手动拆整数/小数
```

---

## 附录：引脚分配表

| 引脚 | 外设 | 功能 |
|------|------|------|
| PA0 | GPIO | 调试 LED（高电平亮） |
| PA2 | SPI1 CS0 | 编码器硬件片选（预留） |
| PA5 | HFXT IN | 40MHz 晶振 |
| PA6 | HFXT OUT | 40MHz 晶振 |
| PA10 | UART0 TX | 调试串口 115200 |
| PA11 | UART0 RX | 调试串口 115200 |
| PA12 | CANFD0 TX | CAN 总线 |
| PA13 | CANFD0 RX | CAN 总线 |
| PA19 | SWDIO | 调试 |
| PA20 | SWCLK | 调试 |
| PA25 | GPIO | 步进驱动器 EN（低电平使能） |
| PA26 | TIMG7 CCP0 | STEP 脉冲 |
| PA27 | GPIO | DIR 方向 |
| PB6 | GPIO | 编码器 CS（手动） |
| PB7 | SPI1 POCI | 编码器 MISO |
| PB8 | SPI1 PICO | 编码器 MOSI |
| PB9 | SPI1 SCLK | 编码器 SCK |

---

## 工程文件结构

```
User/
├── App/
│   ├── main.cpp              ← 多任务入口 (Ctrl+Demo+CAN)，MOTOR_ID 宏
│   ├── main.hpp
│   ├── motor_control.cpp/hpp ← 电机速度/位置控制
│   └── can_protocol.cpp/hpp  ← CAN 多设备协议
├── Bsp/
│   ├── bsp_pwm.cpp/h         ← TIMG7 STEP 脉冲
│   ├── bsp_spi.cpp/h         ← SPI1 编码器通信
│   ├── bsp_can.cpp/h         ← MCAN0 中断收发+滤波
│   ├── bsp_gpio.cpp/h        ← CS/DIR/LED/EN/Delay
│   └── bsp_uart.cpp/h        ← UART0 printf
├── Device/
│   ├── stepper_motor.cpp/h   ← STEP+DIR 驱动
│   └── mt6816.cpp/h          ← 14-bit 磁编码器
├── Module/
│   ├── pid.cpp/h             ← 通用 PID
│   └── speed_measure.cpp/h   ← 角度差分测速
└── Service/
    ├── FreeRTOSConfig.h
    └── sysmem.c
```
