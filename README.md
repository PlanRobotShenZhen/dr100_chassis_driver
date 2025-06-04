# DR100 底盘驱动器

这个包实现了DR100机器人的底盘控制功能，通过串口与下位机通讯来控制机器人运动并获取速度反馈。

## 功能特性

- 监听 `/cmd_vel` 话题接收速度命令
- 通过串口发送控制指令到下位机
- 接收下位机速度反馈数据
- 发布完整的里程计信息到 `/odom` 话题
- 发布TF变换（odom -> base_link）
- 支持速度限制和命令超时保护
- 自动串口重连功能

## 通讯协议

### 上位机发送数据格式（17字节）
- 帧头：0x7B
- 电机使能、急停、车灯控制等
- X/Y/Z轴速度：放大1000倍后发送
- 校验码：异或运算
- 帧尾：0x7D

### 下位机发送数据格式（50字节）
- 帧头：0x7B
- 状态信息
- X/Y/Z轴速度反馈：放大100倍
- 电池、电机状态等信息
- 校验码和帧尾：0x7D

## 使用方法

### 1. 编译
```bash
cd ~/catkin_ws
catkin_make
```

### 2. 启动虚拟串口模拟器（用于测试）
```bash
cd ~/catkin_ws/src/dr100/dr100_chassis_driver
python3 virtual_serial_simulator.py
```

### 3. 启动底盘控制器
```bash
roslaunch dr100_chassis_driver chassis_controller.launch
```

### 4. 发送速度命令
```bash
# 发送前进命令
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 发送转向命令
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"
```

### 5. 查看里程计信息
```bash
# 查看完整的里程计信息
rostopic echo /odom

# 查看TF变换
rosrun tf tf_echo odom base_link
```

### 6. 使用自定义话题名称
如果需要使用不同的话题名称，可以修改launch文件中的参数：
```xml
<!-- 自定义话题名称示例 -->
<param name="cmd_vel_topic" value="/robot/cmd_vel" />
<param name="odom_topic" value="/robot/odom" />
<param name="odom_frame_id" value="robot_odom" />
<param name="base_frame_id" value="robot_base_link" />
```

然后使用自定义话题名称发送命令：
```bash
# 使用自定义话题发送命令
rostopic pub /robot/cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 查看自定义话题的里程计信息
rostopic echo /robot/odom

# 查看自定义TF变换
rosrun tf tf_echo robot_odom robot_base_link
```

## 参数配置

可以在launch文件中修改以下参数：

- `port`: 串口设备路径（默认：/tmp/ttyV1）
- `baudrate`: 波特率（默认：115200）
- `max_linear_velocity`: 最大线速度（默认：2.0 m/s）
- `max_angular_velocity`: 最大角速度（默认：2.0 rad/s）
- `motor_enable`: 电机使能（默认：true）
- `cmd_timeout`: 命令超时时间（默认：1.0秒）
- `cmd_vel_topic`: 速度命令话题名称（默认：/cmd_vel）
- `odom_topic`: 里程计话题名称（默认：/odom）
- `odom_frame_id`: 里程计坐标系名称（默认：odom）
- `base_frame_id`: 机器人基座坐标系名称（默认：base_link）
- `odom_publish_rate`: 里程计发布频率（默认：50.0 Hz，范围：0.1-1000 Hz，≤0时禁用发布）
- `reconnect_interval`: 重连尝试间隔时间（默认：2.0秒）
- `max_reconnect_attempts`: 最大重连尝试次数（默认：-1，表示无限重试）

## 话题

### 订阅的话题
- 速度命令话题 (geometry_msgs/Twist): 可通过`cmd_vel_topic`参数配置，默认为`/cmd_vel`

### 发布的话题
- 里程计话题 (nav_msgs/Odometry): 可通过`odom_topic`参数配置，默认为`/odom`

### 发布的TF变换
- odom -> base_link: 机器人在里程计坐标系中的位置和姿态

## 里程计功能

节点会根据下位机反馈的速度信息计算机器人的位置和姿态：

1. **速度积分**: 使用反馈的线速度和角速度进行数值积分
2. **位置更新**: 根据运动学模型更新机器人在odom坐标系中的位置
3. **协方差矩阵**:
   - 静止时使用高精度协方差矩阵
   - 运动时使用标准协方差矩阵，考虑运动误差
4. **TF发布**: 实时发布odom到base_link的坐标变换
5. **完整里程计**: 发布包含位置、姿态、速度和协方差的完整里程计信息
6. **频率控制**: 可配置里程计发布频率，支持禁用里程计发布功能

## 自动重连功能

当串口连接断开时，节点会自动尝试重连：

1. **断开检测**: 当串口读写操作失败时，自动标记为断开状态
2. **重连间隔**: 每隔`reconnect_interval`秒尝试一次重连
3. **重连次数**: 可通过`max_reconnect_attempts`参数限制重连次数
   - 设置为-1表示无限重试
   - 设置为正数表示最大重试次数
4. **状态提示**: 重连过程中会输出相应的日志信息

### 里程计频率配置示例
```xml
<!-- 高频率里程计发布（100Hz） -->
<param name="odom_publish_rate" value="100.0" />

<!-- 低频率里程计发布（10Hz） -->
<param name="odom_publish_rate" value="10.0" />

<!-- 禁用里程计发布 -->
<param name="odom_publish_rate" value="0" />

<!-- 最低频率（0.1Hz，每10秒发布一次） -->
<param name="odom_publish_rate" value="0.1" />
```

### 重连参数配置示例
```xml
<!-- 每3秒尝试重连一次，最多重试10次 -->
<param name="reconnect_interval" value="3.0" />
<param name="max_reconnect_attempts" value="10" />

<!-- 无限重试，每2秒尝试一次 -->
<param name="reconnect_interval" value="2.0" />
<param name="max_reconnect_attempts" value="-1" />
```

## 注意事项

1. 确保串口设备有正确的读写权限
2. 虚拟串口模拟器仅用于测试，实际使用时需要连接真实的串口设备
3. 速度命令会被限制在设定的最大值范围内
4. 当命令超时时，节点会自动发送停止命令
5. 串口断开时，节点会自动尝试重连，无需手动重启
6. 里程计发布频率会自动限制在0.1-1000Hz范围内，设置为≤0时将禁用里程计发布
7. 即使禁用里程计发布，内部的里程计积分仍会继续进行，只是不发布消息
