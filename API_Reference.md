# MSPM0G3507 步进电机控制平台 — API 参考手册

> 工程：`d:/Ti/test/M0test/m0test`  
> 芯片：MSPM0G3507 (Cortex-M0+, 80MHz, LQFP-48)  
> 架构：Bsp → Device → Module → App 四层

---

## 目录

- [架构概览](#架构概览)
- [App 层 — 应用调度](#app-层)
  - [motor_control — 电机统一控制](#motor_control)
  - [WT901 陀螺仪](#wt901-陀螺仪)
  - [IMU601 姿态模块](#imu601-姿态模块)
  - [can_protocol — CAN 通信协议](#can_protocol)
- [Module 层 — 算法模块](#module-层)
  - [pid — 通用 PID 控制器](#pid)
  - [speed_measure — 编码器测速](#speed_measure)
- [Device 层 — 器件驱动](#device-层)
  - [stepper_motor — 步进电机 STEP+DIR](#stepper_motor)
  - [mt6816 — 磁编码器](#mt6816)
  - [ssd1306 — OLED 显示屏](#ssd1306)
  - [gyro — WT901 协议解析器](#gyro)
  - [imu601 — IMU601 协议解析器](#imu601)
- [Bsp 层 — MCU 硬件抽象](#bsp-层)
  - [bsp_spi / bsp_i2c / bsp_uart / bsp_gpio](#bsp_spi--bsp_i2c--bsp_uart--bsp_gpio)
  - [bsp_pwm — STEP 脉冲](#bsp_pwm)
  - [bsp_can — CAN 总线](#bsp_can)
- [移植指南](#移植指南)
- [附录：引脚分配表](#附录引脚分配表)

---

## 架构概览

```
┌──────────────────────────────────────────────────┐
│  main()                                          │
├──────────────────────────────────────────────────┤
│  App 层   motor_control  can_protocol            │
├──────────────────────────────────────────────────┤
│  Module 层   pid   speed_measure                 │
├──────────────────────────────────────────────────┤
│  Device 层  stepper_motor  mt6816  ssd1306       │
│             gyro (WT901)   imu601 (ICM42688)      │
├──────────────────────────────────────────────────┤
│  Bsp 层    spi  i2c  uart  gpio  pwm  can        │
│            bsp_gyro (WT901 UART1)                 │
│            bsp_imu601 (IMU601 UART2)              │
└──────────────────────────────────────────────────┘
```

**依赖规则**：Bsp 层是唯一 `#include "ti_msp_dl_config.h"` 的地方。  
**换 MCU 移植**：重写 8 个 `bsp_*.cpp`（30 个函数），其余 10 个模块（36 个函数）一字不改。

---

## App 层

上层开发者直接调用的接口。

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

// === 1ms 调度（主循环中调用） ===
void motor_tick(MT6816_Data *enc);          // 控制逻辑，5ms 一次
void motor_measure_tick(MT6816_Data *enc);  // 测速 + 角度记录，1ms 一次
```

**GOTO 到位判定**：误差 < 1° 持续 100ms → 停转，步进保持力矩定住。  
**P 参数**：KP = 2.0（误差角 × 2.0 → 目标 RPM）

---

### WT901 陀螺仪

串口陀螺仪，协议 0x5A 帧头，5 字节帧。Yaw 角 + Z 轴角速度。

**硬件**：UART1, PA9(RX)/PA8(TX), 115200

```c
#include "bsp_gyro.h"

void     bsp_gyro_init(void);      // 初始化 + 开 RX 中断
void     bsp_gyro_poll(void);      // 1ms 消费 ring buffer → 解析
float    bsp_gyro_get_yaw(void);   // Yaw 角 (°)
float    bsp_gyro_get_wz(void);    // Z 轴角速度 (°/s)
uint32_t bsp_gyro_rx_total(void);  // 累计收字节数（调试）
```

---

### IMU601 姿态模块

汇电籽-601 (ICM42688)，协议 AA 55 帧头，12 字节帧。Yaw/Pitch/Roll 三轴姿态。

**硬件**：UART2, PA22(RX)/PA21(TX), 115200  
**上电流程**：软复位 → 500ms → Yaw 校准

```c
#include "bsp_imu601.h"

void     bsp_imu601_init(void);        // 上电复位+校准 + 开中断
void     bsp_imu601_poll(void);        // 1ms 消费 ring buffer → 解析
float    bsp_imu601_get_yaw(void);     // Yaw (°)
float    bsp_imu601_get_pitch(void);   // Pitch (°)
float    bsp_imu601_get_roll(void);    // Roll (°)
uint32_t bsp_imu601_rx_total(void);    // 累计收字节数（调试）
```

**校准角度**在 `bsp_imu601.cpp:70`，当前值 `360.3°`，按需修改。

---

### can_protocol

CAN 总线通信协议（250kbps CAN 2.0）。

| 方向 | CAN ID | 内容 |
|------|--------|------|
| 主机→MSPM0 | 0x100 | `[cmd] [param1 i16] [param2 i16]` |
| MSPM0→主机 | 0x200 | `[stat] [angle×100 i16] [speed×10 i16]` |

```c
#include "can_protocol.hpp"

void can_proto_init(void);    // 注册 CAN ISR
void can_proto_tick(void);    // 收到指令 → 执行 → 50ms 回报状态
```

---

## Module 层

纯算法模块，零硬件依赖，可独立单元测试。

### pid

通用位置式 PID 控制器，带积分抗饱和 (back-calculation) 和输出限幅。

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

编码器角度差分测速。10ms 窗口，低通滤波 (α=0.5)。

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

器件驱动，只依赖 Bsp 层头文件，不碰 `ti_msp_dl_config.h`。换 MCU 一字不改。

### stepper_motor

步进电机 STEP+DIR 驱动。RPM → 脉冲频率换算。

```c
#include "stepper_motor.h"

#define STEPS_PER_REV 6400   // 200 步/圈 × 32 细分

void stepper_init(void);               // DIR=正转, 停脉冲
void stepper_set_speed(float rpm);     // >0 正转, <0 反转, |rpm|<0.5 停
```

---

### mt6816

MT6816 14 位磁编码器，SPI Mode 3，8MHz。

```c
#include "mt6816.h"

typedef struct {
    int     mag;
    uint8_t raw_h, raw_l;
    int     raw_angle;   // 0~16383
    float   angle;       // 0°~360°
} MT6816_Data;

void mt6816_init(void);                  // 拉高 CS
void mt6816_read(MT6816_Data *data);     // 读一次角度（含毛刺过滤）
```

**毛刺过滤**：突然跳到 0 且跳幅 >90° 时拒更新。

---

### ssd1306

SSD1306 OLED 128×64，硬件 I2C 500kHz，地址 0x3C。

```c
#include "ssd1306.h"

void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_set_cursor(uint8_t x, uint8_t y);
void ssd1306_putc(char c);
void ssd1306_puts(const char *s);
void ssd1306_printf(uint8_t x, uint8_t y, const char *fmt, ...);
```

**限制**：TX FIFO 8 字节，每事务只发 2 字节。newlib-nano 不支持 `%f`。

---

### gyro

WT901/JY901 协议解析器——纯数据层，零硬件依赖。

```c
#include "gyro.h"

typedef struct {
    float wz;     // Z 轴角速度 (°/s)，±2000°/s
    float yaw;    // Yaw 角度 (°)，-180°~180°
} GyroData;

void     gyro_init(void);
void     gyro_feed_byte(uint8_t byte);   // 喂一个字节 → 内部拼帧校验
int      gyro_has_new_data(void);        // 有新数据返回 1
GyroData gyro_get_data(void);            // 取最新数据
```

---

### imu601

汇电籽-601 协议解析器——纯数据层，零硬件依赖。

```c
#include "imu601.h"

typedef struct {
    float yaw;    // Yaw (°)，精度 0.01°
    float pitch;  // Pitch (°)
    float roll;   // Roll (°)
} IMU601_Data;

void       imu601_init(void);
void       imu601_feed_byte(uint8_t byte);  // 喂字节 → 状态机拼 12 字节帧
int        imu601_has_new_data(void);
IMU601_Data imu601_get_data(void);
```

---

## Bsp 层

MCU 硬件抽象——**换芯片唯一需要重写的层**。所有文件 `#include "ti_msp_dl_config.h"`。

### bsp_spi

```c
void     bsp_spi_init(void);                  // SPI1 已在 SYSCFG_DL_init 中配置
uint8_t  bsp_spi_transfer(uint8_t tx);        // 全双工单字节
```

### bsp_i2c

```c
void bsp_i2c_send(uint8_t addr, const uint8_t *buf, uint8_t len);  // 最多 8 字节
```

### bsp_uart

```c
int bsp_uart_printf(const char *fmt, ...);    // 阻塞式 printf，newlib-nano 不支持 %f
```

### bsp_gpio

```c
void bsp_cs_high(void);       // PB6 → 高（编码器 CS）
void bsp_cs_low(void);        // PB6 → 低
void bsp_dir_high(void);      // PA27 → 高（电机正转）
void bsp_dir_low(void);       // PA27 → 低（电机反转）
void bsp_led_toggle(void);    // PA14 LED 翻转
void bsp_delay_us(uint32_t us);
```

### bsp_pwm

```c
void bsp_step_pwm_init(void);              // 初始化 TIMG7，1MHz，停计数器
void bsp_step_pwm_set_freq(uint32_t hz);   // 0=停，其他=设频率并启动
                                           // 内部 __disable_irq 保护寄存器原子更新
```

### bsp_can

```c
void bsp_can_init(void);
bool bsp_can_send(uint32_t id, uint8_t *data, uint8_t len);   // 标准帧，≤8 字节
bool bsp_can_recv(uint32_t *id, uint8_t *data, uint8_t *len); // 非阻塞
bool bsp_can_has_msg(void);
```

### bsp_gyro (WT901 UART1)

```c
// 函数签名见 App 层 WT901 部分
// 内部：UART1 ISR + ring buffer → gyro_feed_byte → GyroData
```

### bsp_imu601 (IMU601 UART2)

```c
// 函数签名见 App 层 IMU601 部分
// 内部：UART2 ISR + ring buffer → imu601_feed_byte → IMU601_Data
```

---

## 移植指南

换 MCU（如 STM32）需要做的事：

| 步骤 | 内容 | 文件数 |
|------|------|:--:|
| 1 | 重写 Bsp 层 8 个模块 | 8 `.cpp` |
| 2 | 重写中断服务函数（命名/优先级适配） | 在 Bsp 中 |
| 3 | `main.cpp` 改 `SYSCFG_DL_init()` → 新 MCU 的初始化 | 1 `.cpp` |
| 4 | `CMakeLists.txt` 改 SDK 路径 | 1 |


| ✗ 不需要改 |
|:--|
| Device 层 (5 个模块) — 只依赖 Bsp 头文件接口 |
| Module 层 (2 个模块) — 纯 C 算法 |
| App 层 motor_control / can_protocol — 只依赖下层接口 |
| 任何 `.h` 头文件 |


---

## 附录：引脚分配表

| 引脚 | 外设 | 功能 |
|------|------|------|
| PA0 | I2C0 SDA | OLED |
| PA1 | I2C0 SCL | OLED |
| PA5 | HFXT IN | 40MHz 晶振 |
| PA6 | HFXT OUT | 40MHz 晶振 |
| PA8 | UART1 TX | WT901 陀螺仪 |
| PA9 | UART1 RX | WT901 陀螺仪 |
| PA10 | UART0 TX | 调试串口 |
| PA11 | UART0 RX | 调试串口 |
| PA12 | CANFD0 TX | CAN 总线 |
| PA13 | CANFD0 RX | CAN 总线 |
| PA14 | GPIO | LED |
| PA19 | SWDIO | 调试 |
| PA20 | SWCLK | 调试 |
| PA21 | UART2 TX | IMU601 |
| PA22 | UART2 RX | IMU601 |
| PA26 | TIMG7 CCP0 | STEP 脉冲 |
| PA27 | GPIO | DIR 方向 |
| PB6 | GPIO | 编码器 CS |
| PB7 | SPI1 MISO | 编码器 |
| PB8 | SPI1 MOSI | 编码器 |
| PB9 | SPI1 SCLK | 编码器 |
