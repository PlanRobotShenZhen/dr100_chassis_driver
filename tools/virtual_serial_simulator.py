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

        # 基础状态
        'motor_enable': 0x01,       # 电机使能：0关电机，1开电机
        'fault_info': 0x00,         # 故障信息：0正常，1单电机故障，2多电机故障
        'robot_status': 0x01,       # 机器人系统状态：0停止/待机，1移动，2故障，3急停

        # 设备开关状态（用于模拟实际设备状态，不参与数据包打包）
        'light_state': 0x00,        # 车灯状态：0关闭，1开启
        'ultrasonic_state': 0x00,   # 超声波状态：0关闭，1开启
        'charge_state': 0x00,       # 充电状态：0关闭，1开启
        'lidar_state': 0x00,        # 雷达状态：0关闭，1开启

        # 速度信息（实际物理值，打包时自动放大100倍）
        'x_velocity': 0.0,          # X轴线速度 m/s
        'y_velocity': 0.0,          # Y轴线速度 m/s
        'z_velocity': 0.0,          # Z轴角速度 rad/s

        # 电池信息（实际物理值，打包时自动放大100倍）
        'battery_voltage': 0,       # 电池电压 V
        'battery_current': 0,       # 电池电流 A
        'battery_level': 0,         # 电池电量 %
        'battery_temp': 0,          # 电池温度 ℃

        # 电机状态：0待机，1运行，2故障，3掉线
        'motor_status': [0x01, 0x01, 0x01, 0x01],
        # 电机故障信息：每个位代表一种故障可能
        'motor_faults': [0x0000, 0x0000, 0x0000, 0x0000],
        # 电机脉冲频率 pul/s（32位有符号整数）
        'motor_pulses': [-6149, -6133, 6149, 6149],

        # 里程计（实际物理值m，打包时自动放大100倍）
        'odometry': 0.0
    }


def parse_control_packet(data):
    """解析上位机发送的控制数据包"""
    if len(data) != 17 or data[0] != 0x7B or data[16] != 0x7D:
        return None

    # 验证校验码
    checksum = 0
    for i in range(15):
        checksum ^= data[i]
    if data[15] != checksum:
        print(f"校验码错误: 期望 0x{checksum:02X}, 收到 0x{data[15]:02X}")
        return None

    # 输出接收到的数据包（总是输出，用于调试）
    hex_str = ' '.join(f'{byte:02X}' for byte in data)
    print(f"接收到的数据包: {hex_str}")

    # 解析速度数据
    x_vel, y_vel, z_vel = struct.unpack('<hhh', data[4:10])

    return {
        'motor_enable': data[1],
        'emergency_stop': data[2],
        'light_control': data[3],
        'x_velocity': x_vel / 1000.0,
        'y_velocity': y_vel / 1000.0,
        'z_velocity': z_vel / 1000.0,
        'ultrasonic_switch': data[10],
        'charge_switch': data[11],
        'lidar_switch': data[12]
    }


def pack_data_frame(data_frame):
    """将数据帧字典打包成二进制格式"""
    data = bytearray([data_frame['frame_header']])

    # 基础状态
    data.extend([
        data_frame['motor_enable'],
        data_frame['fault_info'],
        data_frame['robot_status']
    ])

    # 速度信息 (放大100倍)
    data.extend(struct.pack('<hhh',
        int(data_frame['x_velocity'] * 100),
        int(data_frame['y_velocity'] * 100),
        int(data_frame['z_velocity'] * 100)
    ))

    # 电池信息
    data.extend(struct.pack('<HhBb',
        int(data_frame['battery_voltage'] * 100),
        int(data_frame['battery_current'] * 100),
        data_frame['battery_level'],
        data_frame['battery_temp']
    ))

    # 电机状态
    data.extend(data_frame['motor_status'])

    # 电机故障和脉冲
    for i in range(4):
        data.extend(struct.pack('<H', data_frame['motor_faults'][i]))
        pulse = data_frame['motor_pulses'][i]
        data.extend(struct.pack('<HH', pulse & 0xFFFF, (pulse >> 16) & 0xFFFF))

    # 里程计
    odometry = int(data_frame['odometry'] * 100)
    data.extend(struct.pack('<HH', odometry & 0xFFFF, (odometry >> 16) & 0xFFFF))

    # 校验和
    checksum = 0
    for byte in data:
        checksum ^= byte
    data.extend([checksum, data_frame['frame_tail']])

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
    def __init__(self, baudrate=115200, show_packets=False):
        self.create_virtual_serial_ports()
        self.serial_port = serial.Serial(
            port=self.virtual_port_master,
            baudrate=baudrate,
            timeout=1
        )
        atexit.register(self.cleanup)

        self.data_frame = create_robot_data_frame()
        self.receive_buffer = bytearray()
        self.last_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_feedback_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.show_packets = show_packets
        
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
        if self.serial_port.in_waiting <= 0:
            return

        try:
            self.receive_buffer.extend(self.serial_port.read(self.serial_port.in_waiting))

            while len(self.receive_buffer) >= 17:
                # 寻找帧头
                header_index = next((i for i in range(len(self.receive_buffer) - 16)
                                   if self.receive_buffer[i] == 0x7B), -1)

                if header_index == -1:
                    self.receive_buffer.clear()
                    break

                if header_index > 0:
                    self.receive_buffer = self.receive_buffer[header_index:]

                if len(self.receive_buffer) >= 17:
                    packet_data = bytes(self.receive_buffer[:17])
                    self.receive_buffer = self.receive_buffer[17:]

                    control_data = parse_control_packet(packet_data)
                    if control_data:
                        self.update_velocity(control_data)
                        self.update_switches(control_data)
                else:
                    break
        except Exception as e:
            print(f"处理接收数据时出错: {e}")

    def update_velocity(self, control_data):
        """更新速度信息"""
        velocity_changed = any(
            abs(control_data[f'{axis}_velocity'] - self.last_velocity[axis]) > 0.001
            for axis in ['x', 'y', 'z']
        )

        for axis in ['x', 'y', 'z']:
            self.data_frame[f'{axis}_velocity'] = control_data[f'{axis}_velocity']

        if velocity_changed:
            print(f"接收到速度命令: x={control_data['x_velocity']:.3f}, "
                  f"y={control_data['y_velocity']:.3f}, z={control_data['z_velocity']:.3f}")
            self.last_velocity.update({
                'x': control_data['x_velocity'],
                'y': control_data['y_velocity'],
                'z': control_data['z_velocity']
            })

    def update_switches(self, control_data):
        """更新设备开关状态"""
        switches = [
            ('motor_enable', 'motor_enable', '电机使能'),
            ('light_control', 'light_state', '车灯'),
            ('ultrasonic_switch', 'ultrasonic_state', '超声波'),
            ('charge_switch', 'charge_state', '充电'),
            ('lidar_switch', 'lidar_state', '雷达')
        ]

        switch_status = []
        for cmd_key, state_key, name in switches:
            cmd_value = control_data[cmd_key]
            if cmd_value == 1 and self.data_frame[state_key] != 0x01:
                self.data_frame[state_key] = 0x01
                switch_status.append(f"{name}:开启")
            elif cmd_value == 2 and self.data_frame[state_key] != 0x00:
                self.data_frame[state_key] = 0x00
                switch_status.append(f"{name}:关闭")

        if switch_status:
            print(f"设备控制: {', '.join(switch_status)}")

    def generate_frame(self):
        """生成数据帧"""
        # 根据速度更新机器人状态
        self.data_frame['robot_status'] = 0x01 if (
            abs(self.data_frame['x_velocity']) > 0.001 or
            abs(self.data_frame['z_velocity']) > 0.001
        ) else 0x00

        return pack_data_frame(self.data_frame)

    def run(self):
        """运行模拟器"""
        print("虚拟串口模拟器已启动，等待上位机连接...")
        print("按Ctrl+C停止...")

        try:
            while True:
                self.process_received_data()

                # 生成并发送反馈数据帧
                frame = self.generate_frame()
                self.serial_port.write(frame)

                # 输出发送的数据包（可选）
                if self.show_packets:
                    hex_str = ' '.join(f'{byte:02X}' for byte in frame)
                    print(f"发送的数据包: {hex_str}")

                # 检查速度反馈变化
                if any(abs(self.data_frame[f'{axis}_velocity'] - self.last_feedback_velocity[axis]) > 0.001
                       for axis in ['x', 'y', 'z']):
                    print(f"发送速度反馈: x={self.data_frame['x_velocity']:.3f}, "
                          f"y={self.data_frame['y_velocity']:.3f}, z={self.data_frame['z_velocity']:.3f}")

                    for axis in ['x', 'y', 'z']:
                        self.last_feedback_velocity[axis] = self.data_frame[f'{axis}_velocity']

                time.sleep(0.1)
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
    parser.add_argument('--show-packets', action='store_true',
                        help='显示所有发送的数据包（默认只显示接收的数据包）')

    args = parser.parse_args()

    simulator = VirtualSerialSimulator(baudrate=args.baudrate, show_packets=args.show_packets)

    # 应用命令行参数
    if args.velocity:
        set_velocity(simulator.data_frame, args.velocity[0], args.velocity[1], args.velocity[2])

    if args.battery:
        set_battery(simulator.data_frame, args.battery[0], args.battery[1],
                   int(args.battery[2]), int(args.battery[3]))

    simulator.run()