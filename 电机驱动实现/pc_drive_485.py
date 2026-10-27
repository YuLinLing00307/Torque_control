# -*- coding: utf-8 -*-
"""
MXBS-14 关节模组 · 最小驱动脚本（PC + USB转RS485）

接线（模组 8pin 排针）：
    PIN6  DBG_485A  -> USB转485 的 A
    PIN7  DBG_485B  -> USB转485 的 B
    PIN5  GND       -> USB转485 的 GND  （必接，否则通讯不稳）
    PIN1  STO_IN    -> 使能所需电平（见 README，不接电机不转）
    PIN2  STO_COM   -> 对应地

用法：
    pip install pyserial
    python pc_drive_485.py COM5
"""

import sys
import time
import struct

import serial

BAUD = 921600
PAD = 64            # 有效数据填充到 64 字节，再追加 1 字节 CRC8 => 共 65 字节

# ---------------- 协议层 ----------------

def crc8(data):
    """与上位机 utils/comm_functions.py 完全一致的 CRC8"""
    crc = 0
    for d in data:
        crc ^= d
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xB8) if (crc & 0x01) else (crc >> 1)
    return crc


class Joint:
    """OD 直读直写（私有帧，跑在 RS485 上，非 Modbus、非 CANopen）"""

    def __init__(self, port):
        self.s = serial.Serial(port, BAUD, bytesize=8, parity='N',
                               stopbits=1, timeout=0.1)
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def close(self):
        self.s.close()

    def _xfer(self, is_write, index, sub, value, width):
        header = 0x23 if is_write else 0x40
        if is_write:
            payload = bytes([header, index & 0xFF, (index >> 8) & 0xFF, sub]) + \
                      value.to_bytes(width, 'little', signed=True)
        else:
            payload = bytes([header, index & 0xFF, (index >> 8) & 0xFF, sub])

        frame = bytearray(payload)
        frame += b'\x00' * (PAD - len(frame))
        frame.append(crc8(list(frame)))

        self.s.reset_input_buffer()
        time.sleep(0.001)                 # 485 收发切换留时间
        self.s.write(bytes(frame))

        # 应答固定：前 4 字节 echo(header,idx_lo,idx_hi,sub) + 后 4 字节数据(小端)
        buf = b''
        t0 = time.time()
        while len(buf) < 8 and time.time() - t0 < 0.15:
            chunk = self.s.read(self.s.in_waiting or 1)
            if chunk:
                buf += chunk
        if len(buf) < 8:
            raise IOError('无应答（检查接线 / STO / 波特率 / 485 A-B 是否接反）')
        return buf[4:8]

    # ---- 基础读写 ----
    def write(self, index, sub, value, width):
        self._xfer(True, index, sub, value, width)

    def read(self, index, sub, width):
        raw = self._xfer(False, index, sub, 0, width)
        return int.from_bytes(raw[:width], 'little', signed=True)

    # ---- CiA402 常用对象 ----
    def statusword(self):     return self.read(0x6041, 0, 2)
    def controlword(self, v): self.write(0x6040, 0, v, 2)
    def set_mode(self, m):    self.write(0x6060, 0, m, 1)
    def get_mode(self):       return self.read(0x6061, 0, 1)
    def actual_pos(self):     return self.read(0x6064, 0, 4)
    def actual_vel(self):     return self.read(0x606C, 0, 4)
    def actual_torque(self):  return self.read(0x6077, 0, 2)
    def target_pos(self, v):  self.write(0x607A, 0, v, 4)
    def target_vel(self, v):  self.write(0x60FF, 0, v, 4)

    def state_name(self, sw):
        m = sw & 0x6F
        return {
            0x21: 'Ready to switch on', 0x23: 'Switched on',
            0x27: 'Operation enabled',  0x07: 'Quick stop / 未使能',
            0x0F: 'Fault reaction active', 0x08: 'Fault',
        }.get(m, 'Estado 0x%02X' % sw)

    # ---- 状态机 ----
    def enable(self):
        """Shutdown -> Switch on -> Enable operation"""
        sw = self.statusword()
        if (sw & 0x4F) == 0x08:                       # 有故障先复位
            print('  检测到故障，发故障复位 0x80')
            self.controlword(0x0080)
            time.sleep(0.2)
        self.controlword(0x0006); time.sleep(0.05)
        self.controlword(0x0007); time.sleep(0.05)
        self.controlword(0x000F); time.sleep(0.05)
        sw = self.statusword()
        ok = (sw & 0x6F) == 0x27
        print('  使能%s，状态字=0x%04X (%s)' % ('成功' if ok else '失败', sw, self.state_name(sw)))
        return ok

    def disable(self):
        self.controlword(0x0006)          # Shutdown，撤掉力矩输出


# ---------------- 应用层 ----------------

def show_info(j):
    print('\n--- 设备信息 ---')
    try:
        print('  0x1000 产品码  : 0x%08X' % (j.read(0x1000, 0, 4) & 0xFFFFFFFF))
        print('  0x100A 软版本  : %s' % j.read(0x100A, 0, 4))
    except Exception as e:
        print('  读取失败：', e)
    print('\n--- 单位换算相关（决定位置指令怎么算）---')
    try:
        inc = j.read(0x608F, 1, 4)      # 编码器分辨率
        rev = j.read(0x608F, 2, 4)      # 电机转数
        m_rev = j.read(0x6091, 1, 4)    # 齿轮比：电机转数
        o_rev = j.read(0x6091, 2, 4)    # 齿轮比：输出轴转数
        print('  0x608F 编码器分辨率 = %d / %d 转' % (inc, rev))
        print('  0x6091 齿轮比       = %d : %d' % (m_rev, o_rev))
        if inc:
            print('  >> 输出轴 1 圈 ≈ %d 个位置单位' % (inc * m_rev // max(o_rev, 1)))
    except Exception as e:
        print('  读取失败：', e)


def show_status(j):
    sw = j.statusword()
    print('\n--- 状态 ---')
    print('  状态字 0x6041 = 0x%04X  (%s)' % (sw, j.state_name(sw)))
    print('  模式   0x6060 = %d (回读 %d)' % (0, j.get_mode()))
    print('  实际位置      = %d' % j.actual_pos())
    print('  实际速度      = %d' % j.actual_vel())
    print('  实际力矩      = %d' % j.actual_torque())


def move_position(j, delta):
    """位置模式(1)下相对转动 delta 个位置单位，速度限制在起步值"""
    if not j.enable():
        return
    j.set_mode(1)
    j.write(0x6081, 0, 2000, 4)        # profile velocity（保守值）
    j.write(0x6083, 0, 2000, 4)        # profile acceleration
    start = j.actual_pos()
    tgt = start + delta
    print('  起点=%d  目标=%d' % (start, tgt))
    j.target_pos(tgt)
    j.write(0x6040, 0, 0x001F)         # bit4 上升沿触发（从 0x0F -> 0x1F）
    for _ in range(60):
        time.sleep(0.1)
        print('    现在=%d  状态字=0x%04X' % (j.actual_pos(), j.statusword()))
        if abs(j.actual_pos() - tgt) < 50:
            print('  到位')
            break
    j.disable()


def move_velocity(j, rpm_out):
    """速度模式(3)：先使能再给速度（指导书要求：速度模式先发送再使能时另说，此处用先使能）"""
    if not j.enable():
        return
    j.set_mode(3)
    speed = int(rpm_out)
    print('  目标速度(OD 单位)=%d，先给低速 5 秒' % speed)
    j.target_vel(speed)
    try:
        for _ in range(50):
            time.sleep(0.1)
            print('    速度反馈=%d' % j.actual_vel())
    except KeyboardInterrupt:
        print('  用户中断')
    finally:
        j.target_vel(0)
        j.disable()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    print('打开 %s @ %d ...' % (port, BAUD))
    j = Joint(port)

    menu = """
======== MXBS-14 最小驱动菜单 ========
 1  读设备信息 + 单位换算参数
 2  读状态（状态字/位置/速度/力矩）
 3  使能  /  4 失能
 5  位置模式：正转一点（+20000 单位）
 6  位置模式：反转一点（-20000 单位）
 7  速度模式：小速度运行（问答式输入）
 0  退出
"""
    try:
        while True:
            print(menu)
            c = input('选择: ').strip()
            try:
                if c == '1':
                    show_info(j)
                elif c == '2':
                    show_status(j)
                elif c == '3':
                    j.enable()
                elif c == '4':
                    j.disable(); print('  已失能')
                elif c == '5':
                    move_position(j, 20000)
                elif c == '6':
                    move_position(j, -20000)
                elif c == '7':
                    v = int(input('  目标速度 OD 单位（建议 1000 起）: '))
                    move_velocity(j, v)
                elif c == '0':
                    break
            except IOError as e:
                print('  [通讯错误]', e)
            except Exception as e:
                print('  [异常]', type(e).__name__, e)
    finally:
        try:
            j.disable()
        except Exception:
            pass
        j.close()
        print('已断开')


if __name__ == '__main__':
    main()
