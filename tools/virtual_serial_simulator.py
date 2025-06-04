#!/usr/bin/env python3
import serial
import time
import struct
import subprocess
import os
import signal
import atexit


def create_robot_data_frame():
    """创建机器人数据帧字典，存储实际物理值"""
    return {
        # 帧头尾
        'frame_header': 0x7B,
        'frame_tail': 0x7D,

        # 基础数据
        'motor_enable': 0x01,       # 电机使能：bit0:0关电机，1开电机
        'fault_info': 0x00,         # 故障信息：0:正常 1:单电机故障 2：多电机故障
        'robot_status': 0x00,       # 机器人系统状态：0：停止/待机，1：移动，2：故障，3：急停

        # 速度信息 (实际物理值，打包时自动放大100倍) - 默认为0
        'x_velocity': 0.0,          # X轴线速度 m/s
        'y_velocity': 0.0,          # Y轴线速度 m/s
        'z_velocity': 0.0,          # Z轴角速度 rad/s

        # 电池信息 (实际物理值，打包时自动放大100倍)
        'battery_voltage': 24.0,    # 电池电压 V
        'battery_current': 1.0,     # 电池电流 A
        'battery_level': 85,        # 电池电量 %
        'battery_temp': 25,         # 电池温度 ℃

        # 电机状态：0:待机，1：运行，2：故障,3:掉线
        'motor_status': [0x01, 0x01, 0x01, 0x01],

        # 电机故障信息：每个位代表一种故障可能
        'motor_faults': [0x0000, 0x0000, 0x0000, 0x0000],

        # 电机脉冲频率 pul/s (32位有符号整数)
        'motor_pulses': [0, 0, 0, 0],

        # 里程计 (实际物理值m，打包时自动放大100倍) - 保持为0
        'odometry': 0.0
    }


def parse_control_packet(data):
    """解析上位机发送的控制数据包"""
    if len(data) != 17:
        return None

    # 验证帧头和帧尾
    if data[0] != 0x7B or data[16] != 0x7D:
        return None

    # 验证校验码
    checksum = 0
    for i in range(15):
        checksum ^= data[i]

    if data[15] != checksum:
        print(f"校验码错误: 期望 0x{checksum:02X}, 收到 0x{data[15]:02X}")
        return None

    # 解析速度数据（放大1000倍的int16）
    x_velocity_raw = struct.unpack('<h', data[4:6])[0]
    y_velocity_raw = struct.unpack('<h', data[6:8])[0]
    z_velocity_raw = struct.unpack('<h', data[8:10])[0]

    # 转换为实际物理值（除以1000）
    x_velocity = x_velocity_raw / 1000.0
    y_velocity = y_velocity_raw / 1000.0
    z_velocity = z_velocity_raw / 1000.0

    return {
        'motor_enable': data[1],
        'emergency_stop': data[2],
        'light_control': data[3],
        'x_velocity': x_velocity,
        'y_velocity': y_velocity,
        'z_velocity': z_velocity
    }


def pack_data_frame(data_frame):
    """将数据帧字典打包成二进制格式，按照CSV协议规范"""
    data = bytearray()

    # 帧头 (0x7B)
    data.append(data_frame['frame_header'])

    # 电机使能、故障信息、系统状态
    data.append(data_frame['motor_enable'])
    data.append(data_frame['fault_info'])
    data.append(data_frame['robot_status'])

    # 速度信息 (放大100倍后打包为short)
    x_vel_scaled = int(data_frame['x_velocity'] * 100)
    y_vel_scaled = int(data_frame['y_velocity'] * 100)
    z_vel_scaled = int(data_frame['z_velocity'] * 100)
    data.extend(struct.pack('<hhh', x_vel_scaled, y_vel_scaled, z_vel_scaled))

    # 电池信息
    battery_voltage_scaled = int(data_frame['battery_voltage'] * 100)  # 放大100倍
    battery_current_scaled = int(data_frame['battery_current'] * 100)  # 放大100倍
    data.extend(struct.pack('<HhBb',
                           battery_voltage_scaled,
                           battery_current_scaled,
                           data_frame['battery_level'],
                           data_frame['battery_temp']))

    # 电机状态 (4个字节)
    for status in data_frame['motor_status']:
        data.append(status)

    # 电机故障信息和脉冲数 (每个电机6字节：2字节故障+4字节脉冲)
    for i in range(4):
        # 电机故障信息 (u16)
        data.extend(struct.pack('<H', data_frame['motor_faults'][i]))
        # 电机脉冲数 (u32，按CSV协议分为低字节和高字节)
        pulse = data_frame['motor_pulses'][i]
        pulse_low = pulse & 0xFFFF
        pulse_high = (pulse >> 16) & 0xFFFF
        data.extend(struct.pack('<HH', pulse_low, pulse_high))

    # 里程计 (放大100倍后打包为u32，按CSV协议分为低字节和高字节)
    odometry_scaled = int(data_frame['odometry'] * 100)
    odometry_low = odometry_scaled & 0xFFFF
    odometry_high = (odometry_scaled >> 16) & 0xFFFF
    data.extend(struct.pack('<HH', odometry_low, odometry_high))

    # 计算校验和 (对0-47位进行异或运算)
    checksum = 0
    for byte in data:
        checksum ^= byte

    # 添加校验和和帧尾
    data.append(checksum)
    data.append(data_frame['frame_tail'])

    return bytes(data)


def set_velocity(data_frame, x=None, y=None, z=None):
    """设置速度，单位为实际物理值"""
    if x is not None:
        data_frame['x_velocity'] = x
    if y is not None:
        data_frame['y_velocity'] = y
    if z is not None:
        data_frame['z_velocity'] = z


def set_battery(data_frame, voltage=None, current=None, level=None, temp=None):
    """设置电池信息，电压电流为实际物理值"""
    if voltage is not None:
        data_frame['battery_voltage'] = voltage
    if current is not None:
        data_frame['battery_current'] = current
    if level is not None:
        data_frame['battery_level'] = level
    if temp is not None:
        data_frame['battery_temp'] = temp


def set_motor_status(data_frame, index, status):
    """设置电机状态"""
    if 0 <= index < 4:
        data_frame['motor_status'][index] = status


def set_motor_fault(data_frame, index, fault):
    """设置电机故障信息"""
    if 0 <= index < 4:
        data_frame['motor_faults'][index] = fault


def set_motor_pulse(data_frame, index, pulse):
    """设置电机脉冲频率"""
    if 0 <= index < 4:
        data_frame['motor_pulses'][index] = pulse


class VirtualSerialSimulator:
    def __init__(self, baudrate=115200):
        # 创建虚拟串口对
        self.pty_process = None
        self.virtual_port_master = None
        self.virtual_port_slave = None
        self.create_virtual_serial_ports()

        # 连接到虚拟串口
        self.serial_port = serial.Serial(
            port=self.virtual_port_master,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

        # 注册退出处理函数
        atexit.register(self.cleanup)

        # 创建数据帧字典
        self.data_frame = create_robot_data_frame()

        # 接收缓冲区
        self.receive_buffer = bytearray()
        
    def create_virtual_serial_ports(self):
        """创建虚拟串口对"""
        # 使用socat创建虚拟串口对
        cmd = ["socat", "-d", "-d", "pty,raw,echo=0,link=/tmp/ttyV0", "pty,raw,echo=0,link=/tmp/ttyV1"]
        self.pty_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # 等待串口创建完成
        time.sleep(1)
        
        # 设置串口权限
        os.system("sudo chmod 666 /tmp/ttyV0")
        os.system("sudo chmod 666 /tmp/ttyV1")
        
        self.virtual_port_master = "/tmp/ttyV0"
        self.virtual_port_slave = "/tmp/ttyV1"
        
        print(f"虚拟串口已创建: 主端口 {self.virtual_port_master}, 从端口 {self.virtual_port_slave}")
        print(f"请在您的程序中使用 {self.virtual_port_slave} 作为串口设备")
        
    def cleanup(self):
        """清理资源"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            
        if self.pty_process:
            self.pty_process.terminate()
            self.pty_process.wait(timeout=1)
            
        print("虚拟串口已关闭")

    def process_received_data(self):
        """处理接收到的上位机数据"""
        try:
            # 检查是否有数据可读
            if self.serial_port.in_waiting > 0:
                # 读取所有可用数据
                new_data = self.serial_port.read(self.serial_port.in_waiting)
                self.receive_buffer.extend(new_data)

                # 查找完整的数据包（17字节）
                while len(self.receive_buffer) >= 17:
                    # 寻找帧头
                    header_index = -1
                    for i in range(len(self.receive_buffer) - 16):
                        if self.receive_buffer[i] == 0x7B:
                            header_index = i
                            break

                    if header_index == -1:
                        # 没有找到帧头，清空缓冲区
                        self.receive_buffer.clear()
                        break

                    # 移除帧头之前的数据
                    if header_index > 0:
                        self.receive_buffer = self.receive_buffer[header_index:]

                    # 检查是否有完整的数据包
                    if len(self.receive_buffer) >= 17:
                        packet_data = bytes(self.receive_buffer[:17])
                        self.receive_buffer = self.receive_buffer[17:]

                        # 解析控制数据包
                        control_data = parse_control_packet(packet_data)
                        if control_data:
                            # 更新速度反馈
                            self.data_frame['x_velocity'] = control_data['x_velocity']
                            self.data_frame['y_velocity'] = control_data['y_velocity']
                            self.data_frame['z_velocity'] = control_data['z_velocity']

                            # 更新电机使能状态
                            self.data_frame['motor_enable'] = control_data['motor_enable']

                            print(f"接收到速度命令: x={control_data['x_velocity']:.3f}, "
                                  f"y={control_data['y_velocity']:.3f}, z={control_data['z_velocity']:.3f}")
                    else:
                        break

        except Exception as e:
            print(f"处理接收数据时出错: {e}")

    def generate_frame(self):
        """生成数据帧"""
        # 里程计保持为0，不需要更新

        # 根据速度更新机器人状态
        if abs(self.data_frame['x_velocity']) > 0.001 or abs(self.data_frame['z_velocity']) > 0.001:
            self.data_frame['robot_status'] = 0x01  # 移动
        else:
            self.data_frame['robot_status'] = 0x00  # 停止/待机

        # 打包数据
        return pack_data_frame(self.data_frame)
        
    def run(self):
        """运行模拟器"""
        try:
            print("虚拟串口模拟器已启动，等待上位机连接...")
            print("默认速度为0，将根据接收到的命令更新速度反馈")
            print("按Ctrl+C停止...")

            while True:
                # 处理接收到的上位机数据
                self.process_received_data()

                # 生成并发送反馈数据帧
                frame = self.generate_frame()
                self.serial_port.write(frame)

                # 只在有速度时显示详细信息
                if (abs(self.data_frame['x_velocity']) > 0.001 or
                    abs(self.data_frame['y_velocity']) > 0.001 or
                    abs(self.data_frame['z_velocity']) > 0.001):
                    print(f"发送速度反馈: x={self.data_frame['x_velocity']:.3f}, "
                          f"y={self.data_frame['y_velocity']:.3f}, z={self.data_frame['z_velocity']:.3f}")

                time.sleep(0.1)  # 每100ms发送一次

        except KeyboardInterrupt:
            print("\n停止模拟器")
        finally:
            self.cleanup()


def signal_handler(sig, frame):
    """信号处理函数"""
    print("\n接收到信号，正在退出...")
    import sys
    sys.exit(0)


if __name__ == "__main__":
    import argparse
    import sys

    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    parser = argparse.ArgumentParser(description='机器人数据模拟器（虚拟串口）')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--velocity', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='设置机器人速度 (m/s, m/s, rad/s)')
    parser.add_argument('--battery', type=float, nargs=4, metavar=('V', 'A', 'LEVEL', 'TEMP'),
                        help='设置电池信息 (电压V, 电流A, 电量%, 温度℃)')

    args = parser.parse_args()

    simulator = VirtualSerialSimulator(baudrate=args.baudrate)

    # 应用命令行参数
    if args.velocity:
        set_velocity(simulator.data_frame, args.velocity[0], args.velocity[1], args.velocity[2])

    if args.battery:
        set_battery(simulator.data_frame, args.battery[0], args.battery[1],
                   int(args.battery[2]), int(args.battery[3]))

    simulator.run()