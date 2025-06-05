#!/usr/bin/env python3
"""
测试设备开关修复的脚本
验证开关状态变化时是否会立即发送控制包给下位机
"""

import rospy
from std_msgs.msg import Bool
import time

def test_switch_immediate_update():
    """测试开关状态变化时的立即更新"""
    rospy.init_node('test_device_switch_fix', anonymous=True)
    
    # 创建发布者
    light_pub = rospy.Publisher('/light_switch', Bool, queue_size=1)
    ultrasonic_pub = rospy.Publisher('/ultrasonic_switch', Bool, queue_size=1)
    
    # 等待发布者连接
    rospy.sleep(2.0)
    
    print("开始测试设备开关立即更新功能...")
    print("请观察虚拟串口模拟器的输出，应该能看到设备控制状态的变化")
    print()
    
    # 测试序列
    test_cases = [
        ("开启车灯", lambda: light_pub.publish(Bool(data=True))),
        ("等待1秒", lambda: time.sleep(1.0)),
        ("关闭车灯", lambda: light_pub.publish(Bool(data=False))),
        ("等待1秒", lambda: time.sleep(1.0)),
        ("开启超声波", lambda: ultrasonic_pub.publish(Bool(data=True))),
        ("等待1秒", lambda: time.sleep(1.0)),
        ("关闭超声波", lambda: ultrasonic_pub.publish(Bool(data=False))),
        ("等待1秒", lambda: time.sleep(1.0)),
        ("同时开启车灯和超声波", lambda: [
            light_pub.publish(Bool(data=True)),
            ultrasonic_pub.publish(Bool(data=True))
        ]),
        ("等待1秒", lambda: time.sleep(1.0)),
        ("同时关闭车灯和超声波", lambda: [
            light_pub.publish(Bool(data=False)),
            ultrasonic_pub.publish(Bool(data=False))
        ]),
    ]
    
    for description, action in test_cases:
        print(f"执行: {description}")
        if isinstance(action(), list):
            # 处理同时执行多个动作的情况
            pass
        time.sleep(0.5)  # 短暂等待
    
    print()
    print("测试完成！")
    print("如果修复成功，你应该在虚拟串口模拟器中看到：")
    print("- 每次开关状态变化时都有对应的'设备控制'输出")
    print("- 开关状态立即反映在发送的数据帧中")
    print("- 不需要发送速度命令就能看到开关状态变化")

def test_switch_with_velocity():
    """测试开关状态变化与速度命令的组合"""
    rospy.init_node('test_device_switch_with_velocity', anonymous=True)
    
    from geometry_msgs.msg import Twist
    
    # 创建发布者
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    light_pub = rospy.Publisher('/light_switch', Bool, queue_size=1)
    
    rospy.sleep(2.0)
    
    print("测试开关状态变化与速度命令的组合...")
    print()
    
    # 发送速度命令
    twist = Twist()
    twist.linear.x = 0.5
    print("发送前进速度命令 (0.5 m/s)")
    cmd_vel_pub.publish(twist)
    time.sleep(1.0)
    
    # 在有速度的情况下改变开关状态
    print("在运动状态下开启车灯")
    light_pub.publish(Bool(data=True))
    time.sleep(1.0)
    
    print("在运动状态下关闭车灯")
    light_pub.publish(Bool(data=False))
    time.sleep(1.0)
    
    # 停止运动
    twist.linear.x = 0.0
    print("停止运动")
    cmd_vel_pub.publish(twist)
    time.sleep(1.0)
    
    # 在静止状态下改变开关状态
    print("在静止状态下开启车灯")
    light_pub.publish(Bool(data=True))
    time.sleep(1.0)
    
    print("测试完成！")

if __name__ == '__main__':
    try:
        import sys
        if len(sys.argv) > 1 and sys.argv[1] == 'velocity':
            test_switch_with_velocity()
        else:
            test_switch_immediate_update()
    except rospy.ROSInterruptException:
        print("测试被中断")
