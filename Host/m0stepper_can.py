"""
m0stepper CAN 上位机通信库
===========================
封装 MSPM0 步进电机控制器的 CAN 协议，提供简洁的 Python API。

用法:
    import serial
    from m0stepper_can import M0Stepper

    ser = serial.Serial('COM28', 115200)
    motor = M0Stepper(ser, motor_id=1)

    motor.enable()                    # 使能
    motor.set_speed(60)               # 正转 60 RPM
    motor.move_to(90)                 # 转到 90°
    motor.move_to_multi(90, 3, 60)    # 3圈90°, 限速 60 RPM
    motor.stop()                      # 停止
    motor.disable()                   # 失能

    ang, rpm, turns, state = motor.query()
"""

import struct
from typing import Tuple, Optional

# ============================================================
#  CAN 协议常量
# ============================================================

# 命令码
CMD_SPEED    = 0x01
CMD_POSITION = 0x02
CMD_STOP     = 0x03
CMD_ENABLE   = 0x04
CMD_QUERY    = 0x05

# 查询子码
QUERY_ALL    = 0x00
QUERY_ANGLE  = 0x01
QUERY_SPEED  = 0x02
QUERY_TURNS  = 0x03

# 状态码
STAT_IDLE   = 0
STAT_MOVING = 1
STAT_DONE   = 2
STAT_ERROR  = 3

STAT_NAMES = {0: "IDLE", 1: "MOVING", 2: "DONE", 3: "ERROR"}


# ============================================================
#  CAN 帧编解码
# ============================================================

def can_checksum(data: bytes) -> int:
    """XOR(B0~B6)"""
    c = 0
    for b in data[:7]:
        c ^= b
    return c


def build_can_frame(cmd: int, p1: int = 0, p2: int = 0, p3: int = 0) -> bytes:
    """构建 8 字节 CAN 数据帧，自动算校验和"""
    b = bytearray(8)
    b[0] = cmd
    struct.pack_into('>hhh', b, 1, p1, p2, p3)
    b[7] = can_checksum(b)
    return bytes(b)


def parse_status_frame(data: bytes) -> Tuple[int, float, float, int, int]:
    """解析 8 字节状态帧 → (state, angle_deg, rpm, turns, raw_checksum)"""
    state = data[0]
    ang_x100 = struct.unpack_from('>h', data, 1)[0]
    spd_x10  = struct.unpack_from('>h', data, 3)[0]
    turns    = struct.unpack_from('>h', data, 5)[0]
    return state, ang_x100 / 100.0, spd_x10 / 10.0, turns, data[7]


# ============================================================
#  USB-CAN 适配器串口协议
# ============================================================

def pack_usbcan(can_id: int, data: bytes) -> bytes:
    """
    打包 30 字节 USB-CAN 发送帧

    帧格式 (30 bytes):
      0-1:  55 AA         帧头
      2:    1E            帧长
      3:    01            命令 (发送)
      4-7:  01 00 00 00   发送次数
      8-11: 00 00 00 00   发送间隔
      12:   00            ID 类型 (标准帧)
      13-16:can_id LE     CAN ID
      17:   00            帧类型 (数据)
      18:   len           数据长度
      19:   00            保留
      20:   00            保留
      21-28:data          8 字节 CAN 数据
      29:   CRC           XOR(byte2~byte28)
    """
    assert len(data) == 8, "CAN data must be 8 bytes"
    frame = bytearray(30)
    frame[0]  = 0x55
    frame[1]  = 0xAA
    frame[2]  = 0x1E
    frame[3]  = 0x01
    frame[4]  = 0x01           # 发送 1 次
    # 5-11 默认 0
    frame[12] = 0x00           # 标准帧
    struct.pack_into('<I', frame, 13, can_id)
    frame[17] = 0x00           # 数据帧
    frame[18] = len(data)
    # 19-20 保留
    frame[21:29] = data
    # CRC = XOR(byte2~byte28)
    crc = 0
    for b in frame[2:29]:
        crc ^= b
    frame[29] = crc
    return bytes(frame)


def unpack_usbcan(raw: bytes) -> Optional[Tuple[int, bytes]]:
    """
    解包 USB-CAN 接收帧 → (can_id, data) 或 None

    接收帧格式 (16 bytes):
      0:    AA            帧头
      1:    11            帧长?
      2:    08            数据长度
      3-4:  00 00         保留
      5:    00            ID 类型
      6-9:  can_id LE     CAN ID
      10-17:data          8 字节
      18:   55            帧尾
    """
    if len(raw) < 16:
        return None
    if raw[0] != 0xAA or raw[-1] != 0x55:
        return None
    can_id = struct.unpack_from('<I', raw, 6)[0]
    data   = raw[10:18]
    return can_id, data


# ============================================================
#  上位机 API
# ============================================================

class M0Stepper:
    """MSPM0 步进电机 CAN 控制器"""

    def __init__(self, serial_port, motor_id: int = 1):
        """
        serial_port: pyserial Serial 对象 (115200, 8N1)
        motor_id:    电机设备号 (1~N)
        """
        self.ser = serial_port
        self.motor_id = motor_id
        self._cmd_id  = 0x100 + motor_id
        self._stat_id = 0x200 + motor_id

    # ---- 底层收发 ----

    def _send(self, data: bytes):
        """发送 CAN 数据帧到电机"""
        frame = pack_usbcan(self._cmd_id, data)
        self.ser.write(frame)

    def _recv(self, timeout_ms: int = 100) -> Optional[Tuple[int, bytes]]:
        """接收一帧 CAN 数据 (阻塞)"""
        self.ser.timeout = timeout_ms / 1000.0
        # USB-CAN 接收帧可能混在流里，需要读够 16 字节
        # 简单策略：读到 AA 头再读 15 字节
        while True:
            h = self.ser.read(1)
            if not h:
                return None
            if h[0] == 0xAA:
                rest = self.ser.read(15)
                if len(rest) == 15:
                    return unpack_uscan(b'\xAA' + rest)
        return None

    # ---- 命令 ----

    def set_speed(self, rpm: float):
        """
        速度控制 (开环)

        Args:
            rpm: 目标转速，正=正转，负=反转
        """
        p1 = int(rpm * 10)
        self._send(build_can_frame(CMD_SPEED, p1))

    def move_to(self, angle_deg: float, max_rpm: float = 0):
        """
        单圈位置控制 (闭环 P)

        Args:
            angle_deg: 目标角度 0°~360°
            max_rpm:   最大转速 (0=默认45RPM)
        """
        self.move_to_multi(angle_deg, turns=0, max_rpm=max_rpm)

    def move_to_multi(self, angle_deg: float, turns: int = 0, max_rpm: float = 45):
        """
        多圈位置控制 (闭环 P)

        Args:
            angle_deg: 目标角度 0°~360°
            turns:     目标圈数 (可正可负)
            max_rpm:   最大转速 RPM
        """
        p1 = int(angle_deg * 100)
        p2 = turns
        p3 = int(max_rpm)
        self._send(build_can_frame(CMD_POSITION, p1, p2, p3))

    def stop(self):
        """立即停止"""
        self._send(build_can_frame(CMD_STOP))

    def enable(self):
        """使能驱动器 (PA25 拉低)"""
        self._send(build_can_frame(CMD_ENABLE, p1=1))

    def disable(self):
        """失能驱动器 (PA25 拉高)"""
        self._send(build_can_frame(CMD_ENABLE, p1=0))

    def query(self, sub: int = QUERY_ALL) -> Tuple[float, float, int, int]:
        """
        查询状态

        Args:
            sub: QUERY_ALL(0)/ANGLE(1)/SPEED(2)/TURNS(3)

        Returns:
            (angle_deg, rpm, turns, state)
            仅部分字段有效时其余字段为 0
        """
        self._send(build_can_frame(CMD_QUERY, p1=sub))
        result = self._recv(timeout_ms=200)
        if result:
            can_id, data = result
            ck = can_checksum(data)
            if data[7] == ck:
                state, ang, rpm, turns, _ = parse_status_frame(data)
                return ang, rpm, turns, state
        return 0.0, 0.0, 0, -1   # 超时/校验失败返回 -1 state

    def query_state(self) -> str:
        """查询状态 → 'IDLE'/'MOVING'/'DONE'/'ERROR'"""
        _, _, _, state = self.query()
        return STAT_NAMES.get(state, f"UNKNOWN({state})")

    def query_angle(self) -> float:
        """查询当前角度 (°)"""
        ang, _, _, _ = self.query(QUERY_ANGLE)
        return ang

    def query_speed(self) -> float:
        """查询当前转速 (RPM)"""
        _, rpm, _, _ = self.query(QUERY_SPEED)
        return rpm

    def query_turns(self) -> int:
        """查询累计圈数"""
        _, _, turns, _ = self.query(QUERY_TURNS)
        return turns

    def wait_done(self, timeout_s: float = 10.0, poll_ms: int = 50):
        """
        等待 GOTO 到位

        Args:
            timeout_s: 超时时间 (秒)
            poll_ms:   轮询间隔 (毫秒)

        Raises:
            TimeoutError: 超时未到位
        """
        import time
        start = time.time()
        while time.time() - start < timeout_s:
            _, _, _, state = self.query()
            if state == STAT_DONE:
                return
            time.sleep(poll_ms / 1000.0)
        raise TimeoutError(f"GOTO 未到位 (timeout={timeout_s}s)")


# ============================================================
#  快速测试 (直接运行)
# ============================================================

if __name__ == '__main__':
    import sys
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

    print("=== m0stepper CAN Command Frame Test ===\n")

    motor_id = 1
    cmd_id   = 0x100 + motor_id

    tests = [
        ("SPEED +60 RPM",      build_can_frame(CMD_SPEED,    p1=600)),
        ("SPEED +120 RPM",     build_can_frame(CMD_SPEED,    p1=1200)),
        ("SPEED -60 RPM",      build_can_frame(CMD_SPEED,    p1=-600)),
        ("SPEED 0 RPM",        build_can_frame(CMD_SPEED,    p1=0)),
        ("POS 90 deg (default)", build_can_frame(CMD_POSITION, p1=9000, p2=0)),
        ("POS 180 deg max60",  build_can_frame(CMD_POSITION, p1=18000, p3=60)),
        ("POS 3turn+90 max60", build_can_frame(CMD_POSITION, p1=9000, p2=3, p3=60)),
        ("POS -5turn+0 max30", build_can_frame(CMD_POSITION, p1=0, p2=-5, p3=30)),
        ("STOP",               build_can_frame(CMD_STOP)),
        ("ENABLE on",          build_can_frame(CMD_ENABLE,   p1=1)),
        ("ENABLE off",         build_can_frame(CMD_ENABLE,   p1=0)),
        ("QUERY all",          build_can_frame(CMD_QUERY,    p1=0)),
        ("QUERY angle only",   build_can_frame(CMD_QUERY,    p1=1)),
        ("QUERY speed only",   build_can_frame(CMD_QUERY,    p1=2)),
        ("QUERY turns only",   build_can_frame(CMD_QUERY,    p1=3)),
    ]

    for name, can_data in tests:
        usbcan = pack_usbcan(cmd_id, can_data)
        hex_str = ' '.join(f'{b:02X}' for b in usbcan)
        can_hex = ' '.join(f'{b:02X}' for b in can_data)
        ck = can_data[7]
        ck_ok = "OK" if can_checksum(can_data) == ck else "FAIL"
        print(f"[{name}]")
        print(f"  CAN:  {can_hex}  CK={ck:02X} {ck_ok}")
        print(f"  USB-CAN (30B):  {hex_str}")
        print()
