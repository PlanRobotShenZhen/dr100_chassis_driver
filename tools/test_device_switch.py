#!/usr/bin/env python3
"""
测试设备开关功能的脚本
验证开关状态变化时是否会立即发送控制包给下位机
包括车灯、超声波、充电、雷达、急停、电机使能等功能
"""

import rospy
from std_msgs.msg import Bool
import time
import sys

def check_available_topics():
    """检查可用的设备控制话题"""
    print("检查可用的设备控制话题...")

    # 获取所有话题（包括发布者和订阅者）
    import subprocess
    try:
        # 使用rostopic list获取所有话题
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topic_names = result.stdout.strip().split('\n')
            topic_names = [t.strip() for t in topic_names if t.strip()]  # 清理空行
            print(f"找到 {len(topic_names)} 个话题")
        else:
            print("无法获取话题列表，使用rospy方法...")
            # 备用方法：使用rospy
            topics = rospy.get_published_topics()
            topic_names = [topic[0] for topic in topics]
            print(f"找到 {len(topic_names)} 个话题（仅发布者）")
    except (subprocess.TimeoutExpired, FileNotFoundError):
        print("rostopic命令不可用，使用rospy方法...")
        # 备用方法：使用rospy
        topics = rospy.get_published_topics()
        topic_names = [topic[0] for topic in topics]
        print(f"找到 {len(topic_names)} 个话题（仅发布者）")

    # 检查设备控制话题
    device_topics = {
        '/light_switch': 'light',
        '/ultrasonic_switch': 'ultrasonic',
        '/charge_switch': 'charge',
        '/lidar_switch': 'lidar',
        '/emergency_stop': 'emergency',
        '/motor_enable': 'motor_enable'
    }

    available_topics = {}
    for topic, device in device_topics.items():
        if topic in topic_names:
            available_topics[topic] = device
            print(f"  ✓ {device}: {topic}")
        else:
            print(f"  ✗ {device}: {topic} (disabled)")

    print()
    return available_topics

def test_switch_immediate_update():
    """测试开关状态变化时的立即更新"""
    rospy.init_node('test_device_switch_comprehensive', anonymous=True)

    # 等待节点启动
    rospy.sleep(1.0)

    # 检查可用话题
    available_topics = check_available_topics()

    if not available_topics:
        print("错误：没有找到任何设备控制话题！")
        print("请确保chassis_controller节点正在运行，并且至少启用了一个设备控制话题。")
        return

    # 创建发布者（只为可用的话题创建）
    publishers = {}
    for topic, device in available_topics.items():
        publishers[device] = rospy.Publisher(topic, Bool, queue_size=1)

    # 等待发布者连接
    rospy.sleep(2.0)

    print("开始测试设备开关立即更新功能...")
    print("请观察虚拟串口模拟器的输出，应该能看到设备控制状态的变化")
    print()
    
    # 构建测试序列（基于可用的设备）
    test_cases = []

    # 基础设备测试
    if 'light' in publishers:
        test_cases.extend([
            ("开启车灯", lambda: publishers['light'].publish(Bool(data=True))),
            ("等待1秒", lambda: time.sleep(1.0)),
            ("关闭车灯", lambda: publishers['light'].publish(Bool(data=False))),
            ("等待1秒", lambda: time.sleep(1.0)),
        ])

    if 'ultrasonic' in publishers:
        test_cases.extend([
            ("开启超声波", lambda: publishers['ultrasonic'].publish(Bool(data=True))),
            ("等待1秒", lambda: time.sleep(1.0)),
            ("关闭超声波", lambda: publishers['ultrasonic'].publish(Bool(data=False))),
            ("等待1秒", lambda: time.sleep(1.0)),
        ])

    if 'charge' in publishers:
        test_cases.extend([
            ("开启充电", lambda: publishers['charge'].publish(Bool(data=True))),
            ("等待1秒", lambda: time.sleep(1.0)),
            ("关闭充电", lambda: publishers['charge'].publish(Bool(data=False))),
            ("等待1秒", lambda: time.sleep(1.0)),
        ])

    if 'lidar' in publishers:
        test_cases.extend([
            ("开启雷达", lambda: publishers['lidar'].publish(Bool(data=True))),
            ("等待1秒", lambda: time.sleep(1.0)),
            ("关闭雷达", lambda: publishers['lidar'].publish(Bool(data=False))),
            ("等待1秒", lambda: time.sleep(1.0)),
        ])

    # 安全功能测试
    if 'emergency' in publishers:
        test_cases.extend([
            ("触发急停", lambda: publishers['emergency'].publish(Bool(data=True))),
            ("等待1秒", lambda: time.sleep(1.0)),
            ("取消急停", lambda: publishers['emergency'].publish(Bool(data=False))),
            ("等待1秒", lambda: time.sleep(1.0)),
        ])

    if 'motor_enable' in publishers:
        test_cases.extend([
            ("开启电机", lambda: publishers['motor_enable'].publish(Bool(data=True))),
            ("等待1秒", lambda: time.sleep(1.0)),
            ("关闭电机", lambda: publishers['motor_enable'].publish(Bool(data=False))),
            ("等待1秒", lambda: time.sleep(1.0)),
        ])

    # 组合测试（如果有多个设备可用）
    if 'light' in publishers and 'ultrasonic' in publishers:
        test_cases.extend([
            ("同时开启车灯和超声波", lambda: [
                publishers['light'].publish(Bool(data=True)),
                publishers['ultrasonic'].publish(Bool(data=True))
            ]),
            ("等待1秒", lambda: time.sleep(1.0)),
            ("同时关闭车灯和超声波", lambda: [
                publishers['light'].publish(Bool(data=False)),
                publishers['ultrasonic'].publish(Bool(data=False))
            ]),
            ("等待1秒", lambda: time.sleep(1.0)),
        ])
    
    if not test_cases:
        print("没有可用的设备进行测试！")
        return

    for description, action in test_cases:
        print(f"执行: {description}")
        result = action()
        if isinstance(result, list):
            # 处理同时执行多个动作的情况
            pass
        time.sleep(0.5)  # 短暂等待

    print()
    print("测试完成！")
    print("如果功能正常，你应该在虚拟串口模拟器中看到：")
    print("- 每次开关状态变化时都有对应的'设备控制'输出")
    print("- 开关状态立即反映在发送的数据帧中")
    print("- 不需要发送速度命令就能看到开关状态变化")
    print("- 急停和电机使能状态的变化")
    print("- 2秒后开关状态自动重置为保持状态(0)")

def test_switch_with_velocity():
    """测试开关状态变化与速度命令的组合"""
    rospy.init_node('test_device_switch_with_velocity', anonymous=True)

    from geometry_msgs.msg import Twist

    # 等待节点启动
    rospy.sleep(1.0)

    # 检查可用话题
    available_topics = check_available_topics()

    if not available_topics:
        print("错误：没有找到任何设备控制话题！")
        return

    # 创建发布者
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    publishers = {}
    for topic, device in available_topics.items():
        publishers[device] = rospy.Publisher(topic, Bool, queue_size=1)

    rospy.sleep(2.0)

    print("测试开关状态变化与速度命令的组合...")
    print()

    # 发送速度命令
    twist = Twist()
    twist.linear.x = 0.5
    print("发送前进速度命令 (0.5 m/s)")
    cmd_vel_pub.publish(twist)
    time.sleep(1.0)

    # 在有速度的情况下改变开关状态（测试第一个可用设备）
    first_device = list(publishers.keys())[0]
    print(f"在运动状态下开启{first_device}")
    publishers[first_device].publish(Bool(data=True))
    time.sleep(1.0)

    print(f"在运动状态下关闭{first_device}")
    publishers[first_device].publish(Bool(data=False))
    time.sleep(1.0)

    # 测试急停功能（如果可用）
    if 'emergency' in publishers:
        print("在运动状态下触发急停")
        publishers['emergency'].publish(Bool(data=True))
        time.sleep(1.0)

        print("取消急停")
        publishers['emergency'].publish(Bool(data=False))
        time.sleep(1.0)

    # 停止运动
    twist.linear.x = 0.0
    print("停止运动")
    cmd_vel_pub.publish(twist)
    time.sleep(1.0)

    # 在静止状态下改变开关状态
    print(f"在静止状态下开启{first_device}")
    publishers[first_device].publish(Bool(data=True))
    time.sleep(1.0)

    # 测试电机使能功能（如果可用）
    if 'motor_enable' in publishers:
        print("在静止状态下关闭电机")
        publishers['motor_enable'].publish(Bool(data=False))
        time.sleep(1.0)

        print("重新开启电机")
        publishers['motor_enable'].publish(Bool(data=True))
        time.sleep(1.0)

    print("测试完成！")

def test_parameter_configuration():
    """测试参数配置功能"""
    rospy.init_node('test_device_parameters', anonymous=True)

    print("测试设备控制参数配置...")
    print()

    # 等待节点启动
    rospy.sleep(1.0)

    # 检查参数
    param_names = [
        'enable_light', 'enable_ultrasonic', 'enable_charge',
        'enable_lidar', 'enable_emergency', 'enable_motor_enable'
    ]

    print("当前参数配置：")
    for param in param_names:
        try:
            value = rospy.get_param(f'/chassis_controller/{param}', 'not_set')
            status = "✓" if value else "✗"
            print(f"  {status} {param}: {value}")
        except Exception as e:
            print(f"  ? {param}: 无法获取 ({e})")

    print()

    # 检查对应的话题
    available_topics = check_available_topics()

    print("参数与话题对应关系：")
    param_topic_map = {
        'enable_light': '/light_switch',
        'enable_ultrasonic': '/ultrasonic_switch',
        'enable_charge': '/charge_switch',
        'enable_lidar': '/lidar_switch',
        'enable_emergency': '/emergency_stop',
        'enable_motor_enable': '/motor_enable'
    }

    for param, topic in param_topic_map.items():
        try:
            param_value = rospy.get_param(f'/chassis_controller/{param}', None)
            topic_available = topic in [t for t in available_topics.keys()]

            if param_value is True and topic_available:
                print(f"  ✓ {param}=true → {topic} (可用)")
            elif param_value is False and not topic_available:
                print(f"  ✓ {param}=false → {topic} (已禁用)")
            elif param_value is True and not topic_available:
                print(f"  ✗ {param}=true → {topic} (配置错误：参数启用但话题不可用)")
            elif param_value is False and topic_available:
                print(f"  ✗ {param}=false → {topic} (配置错误：参数禁用但话题可用)")
            else:
                print(f"  ? {param} → {topic} (无法确定状态)")
        except Exception as e:
            print(f"  ? {param} → {topic} (检查失败: {e})")

    print()
    print("参数配置测试完成！")

def print_usage():
    """打印使用说明"""
    print("设备开关测试脚本")
    print()
    print("用法:")
    print("  python3 test_device_switch.py [选项]")
    print()
    print("选项:")
    print("  无参数      - 运行基础设备开关测试")
    print("  velocity    - 运行开关与速度命令组合测试")
    print("  params      - 运行参数配置测试")
    print("  help        - 显示此帮助信息")
    print()
    print("示例:")
    print("  python3 test_device_switch.py")
    print("  python3 test_device_switch.py velocity")
    print("  python3 test_device_switch.py params")

if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == 'velocity':
                test_switch_with_velocity()
            elif sys.argv[1] == 'params':
                test_parameter_configuration()
            elif sys.argv[1] == 'help':
                print_usage()
            else:
                print(f"未知选项: {sys.argv[1]}")
                print_usage()
        else:
            test_switch_immediate_update()
    except rospy.ROSInterruptException:
        print("测试被中断")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
