# DR100 底盘驱动器

这个包实现了DR100机器人的底盘控制功能，通过串口与下位机通讯来控制机器人运动并获取速度反馈。

## 代码结构

1. **核心模块结构**
- `chassis_controller.cpp` - 主控制器，负责协调各个模块
- `serial_communication.cpp` - 串口通信模块
- `odometry_publisher.cpp` - 里程计发布模块
- `device_control.cpp` - 设备控制模块（车灯、超声波、充电、雷达、急停、电机使能等）
- `battery_monitor.cpp` - 电池监控模块
- `chassis_status_monitor.cpp` - 底盘状态监控模块

2. **对应的头文件**
- `serial_communication.h`
- `odometry_publisher.h`
- `device_control.h`
- `battery_monitor.h`
- `chassis_status_monitor.h`
- `common_types.h` - 共享的数据结构和常量

## 功能特性

- 监听 `/cmd_vel` 话题接收速度命令
- 通过串口发送控制指令到下位机
- 接收下位机速度反馈数据
- 发布完整的里程计信息到 `/odom` 话题
- 发布TF变换（odom -> base_link）
- 支持速度限制和命令超时保护
- 自动串口重连功能
- 设备控制功能（车灯、超声波、充电、雷达开关）
- 安全控制功能（急停命令、电机使能开关）
- 电池状态监控和发布
- 底盘状态监控（电机使能状态、故障信息、机器人系统状态）

## 通讯协议

### 上位机发送数据格式（17字节）
- 帧头：0x7B
- 电机使能：0保持，1开启，2关闭
- 急停命令：0保持，1急停，2取消急停
- 车灯控制：0保持，1开启，2关闭
- X/Y/Z轴速度：放大1000倍后发送
- 超声波、充电、雷达开关：0保持，1开启，2关闭
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
python3 tools/virtual_serial_simulator.py
```

#### 虚拟串口模拟器高级用法
```bash
# 基本启动
python3 tools/virtual_serial_simulator.py

# 指定波特率
python3 tools/virtual_serial_simulator.py --baudrate 115200

# 设置初始速度
python3 tools/virtual_serial_simulator.py --velocity 0.5 0.0 0.2

# 设置电池信息（电压V，电流A，电量%，温度℃）
python3 tools/virtual_serial_simulator.py --battery 24.0 -2.5 85 25

# 显示所有数据包（用于调试）
python3 tools/virtual_serial_simulator.py --show-packets

# 组合使用
python3 tools/virtual_serial_simulator.py --velocity 0.3 0.0 0.1 --battery 23.8 -1.8 75 28 --show-packets
```

### 3. 启动底盘控制器
```bash
# 使用完整配置启动
roslaunch dr100_chassis_driver chassis_controller.launch

# 使用最小配置启动（仅基本功能）
roslaunch dr100_chassis_driver chassis_controller_minimal.launch
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
- `battery_topic`: 电池状态话题名称（默认：/battery_state）
- `battery_publish_rate`: 电池状态发布频率（默认：10.0 Hz）
- `light_topic`: 车灯开关话题名称（默认：/light_switch）
- `ultrasonic_topic`: 超声波开关话题名称（默认：/ultrasonic_switch）
- `charge_topic`: 充电开关话题名称（默认：/charge_switch）
- `lidar_topic`: 雷达开关话题名称（默认：/lidar_switch）
- `emergency_topic`: 急停命令话题名称（默认：/emergency_stop）
- `motor_enable_topic`: 电机使能话题名称（默认：/motor_enable）
- `enable_light`: 是否启用车灯控制话题（默认：true）
- `enable_ultrasonic`: 是否启用超声波控制话题（默认：true）
- `enable_charge`: 是否启用充电控制话题（默认：true）
- `enable_lidar`: 是否启用雷达控制话题（默认：true）
- `enable_emergency`: 是否启用急停控制话题（默认：true）
- `enable_motor_enable`: 是否启用电机使能控制话题（默认：true）
- `enable_chassis_status`: 是否启用底盘状态监控功能（默认：true）
- `chassis_status_publish_rate`: 底盘状态发布频率（默认：10.0 Hz）
- `chassis_motor_enable_status_topic`: 电机使能状态话题名称（默认：/chassis/motor_enable_status）
- `chassis_fault_status_topic`: 故障状态话题名称（默认：/chassis/fault_status）
- `chassis_robot_status_topic`: 机器人系统状态话题名称（默认：/chassis/robot_status）
- `chassis_diagnostics_topic`: 底盘诊断话题名称（默认：/chassis/diagnostics）

## 话题

### 订阅的话题
- 速度命令话题 (geometry_msgs/Twist): 可通过`cmd_vel_topic`参数配置，默认为`/cmd_vel`
- 车灯开关话题 (std_msgs/Bool): 可通过`light_topic`参数配置，默认为`/light_switch`
- 超声波开关话题 (std_msgs/Bool): 可通过`ultrasonic_topic`参数配置，默认为`/ultrasonic_switch`
- 充电开关话题 (std_msgs/Bool): 可通过`charge_topic`参数配置，默认为`/charge_switch`
- 雷达开关话题 (std_msgs/Bool): 可通过`lidar_topic`参数配置，默认为`/lidar_switch`
- 急停命令话题 (std_msgs/Bool): 可通过`emergency_topic`参数配置，默认为`/emergency_stop`
- 电机使能话题 (std_msgs/Bool): 可通过`motor_enable_topic`参数配置，默认为`/motor_enable`

### 发布的话题
- 里程计话题 (nav_msgs/Odometry): 可通过`odom_topic`参数配置，默认为`/odom`
- 电池状态话题 (sensor_msgs/BatteryState): 可通过`battery_topic`参数配置，默认为`/battery_state`
- 电机使能状态话题 (std_msgs/Bool): 可通过`chassis_motor_enable_status_topic`参数配置，默认为`/chassis/motor_enable_status`
- 故障状态话题 (std_msgs/UInt8): 可通过`chassis_fault_status_topic`参数配置，默认为`/chassis/fault_status`
- 机器人系统状态话题 (std_msgs/UInt8): 可通过`chassis_robot_status_topic`参数配置，默认为`/chassis/robot_status`
- 底盘诊断话题 (diagnostic_msgs/DiagnosticArray): 可通过`chassis_diagnostics_topic`参数配置，默认为`/chassis/diagnostics`

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

## 设备控制功能

节点支持通过ROS话题控制各种设备开关和安全功能，采用三态控制协议。可通过参数控制是否启用各个设备控制话题：

1. **车灯开关**: 通过`/light_switch`话题接收Bool类型的车灯开关命令（可通过`enable_light`参数禁用）
2. **超声波开关**: 通过`/ultrasonic_switch`话题接收Bool类型的超声波开关命令（可通过`enable_ultrasonic`参数禁用）
3. **充电开关**: 通过`/charge_switch`话题接收Bool类型的充电开关命令（可通过`enable_charge`参数禁用）
4. **雷达开关**: 通过`/lidar_switch`话题接收Bool类型的雷达开关命令（可通过`enable_lidar`参数禁用）
5. **急停命令**: 通过`/emergency_stop`话题接收Bool类型的急停命令（可通过`enable_emergency`参数禁用）
6. **电机使能**: 通过`/motor_enable`话题接收Bool类型的电机使能命令（可通过`enable_motor_enable`参数禁用）

### 控制协议
- **设备开关协议**（车灯、超声波、充电、雷达）：
  - **0**: 保持当前状态（默认状态，无话题数据时发送）
  - **1**: 开启设备（Bool话题data=true时发送）
  - **2**: 关闭设备（Bool话题data=false时发送）

- **急停命令协议**：
  - **0**: 保持当前状态
  - **1**: 急停（Bool话题data=true时发送）
  - **2**: 取消急停（Bool话题data=false时发送）

- **电机使能协议**：
  - **0**: 保持当前状态
  - **1**: 开启电机（Bool话题data=true时发送）
  - **2**: 关闭电机（Bool话题data=false时发送）

### 自动状态重置
- 当开关话题超过2秒没有新数据时，自动重置为保持状态(0)
- 确保在话题断开或无数据时不会持续发送开启/关闭命令

### 设备控制命令示例
```bash
# 开启车灯（发送1给下位机）
rostopic pub /light_switch std_msgs/Bool "data: true"

# 关闭车灯（发送2给下位机）
rostopic pub /light_switch std_msgs/Bool "data: false"

# 开启超声波（发送1给下位机）
rostopic pub /ultrasonic_switch std_msgs/Bool "data: true"

# 关闭超声波（发送2给下位机）
rostopic pub /ultrasonic_switch std_msgs/Bool "data: false"

# 开启充电（发送1给下位机）
rostopic pub /charge_switch std_msgs/Bool "data: true"

# 关闭充电（发送2给下位机）
rostopic pub /charge_switch std_msgs/Bool "data: false"

# 开启雷达（发送1给下位机）
rostopic pub /lidar_switch std_msgs/Bool "data: true"

# 关闭雷达（发送2给下位机）
rostopic pub /lidar_switch std_msgs/Bool "data: false"

# 急停命令（发送1给下位机）
rostopic pub /emergency_stop std_msgs/Bool "data: true"

# 取消急停（发送2给下位机）
rostopic pub /emergency_stop std_msgs/Bool "data: false"

# 开启电机（发送1给下位机）
rostopic pub /motor_enable std_msgs/Bool "data: true"

# 关闭电机（发送2给下位机）
rostopic pub /motor_enable std_msgs/Bool "data: false"

# 注意：停止发送话题数据2秒后，所有开关自动重置为保持状态(0)
```

### 设备控制配置示例
可以通过launch文件参数选择性启用设备控制功能：

```xml
<!-- 只启用车灯和急停功能，禁用其他设备控制 -->
<param name="enable_light" value="true" />
<param name="enable_ultrasonic" value="false" />
<param name="enable_charge" value="false" />
<param name="enable_lidar" value="false" />
<param name="enable_emergency" value="true" />
<param name="enable_motor_enable" value="true" />
```

### 设备控制测试
详细的测试方法和配置示例请参考：
- `tools/test_device_switch.py` - 自动化测试脚本
- `launch/chassis_controller_minimal.launch` - 最小配置示例

```bash
# 运行设备控制测试
cd ~/catkin_ws/src/dr100/dr100_chassis_driver
python3 tools/test_device_switch.py

# 查看测试帮助
python3 tools/test_device_switch.py help

# 测试开关状态变化与速度命令的组合
python3 tools/test_device_switch.py velocity

# 测试自动重置功能
python3 tools/test_device_switch.py reset
```

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

## 底盘状态监控功能

节点会监控底盘状态并发布到指定话题：

### 电机使能状态 (motor_enable_status)
- **false**: 电机关闭
- **true**: 电机开启

### 故障状态 (fault_status)
- **0**: 正常
- **1**: 单电机故障
- **2**: 多电机故障

### 机器人系统状态 (robot_status)
- **0**: 停止/待机
- **1**: 移动
- **2**: 故障
- **3**: 急停

### 诊断信息
底盘状态监控模块还会发布详细的诊断信息，包括：
- 当前所有状态的文字描述
- 根据故障和系统状态自动设置的诊断级别（OK/WARN/ERROR）
- 状态变化时的日志记录

### 底盘状态监控配置示例
```xml
<!-- 启用底盘状态监控，发布频率10Hz -->
<param name="enable_chassis_status" value="true" />
<param name="chassis_status_publish_rate" value="10.0" />

<!-- 自定义话题名称 -->
<param name="chassis_motor_enable_status_topic" value="/my_chassis/motor_enable_status" />
<param name="chassis_fault_status_topic" value="/my_chassis/fault_status" />
<param name="chassis_robot_status_topic" value="/my_chassis/robot_status" />
<param name="chassis_diagnostics_topic" value="/my_chassis/diagnostics" />
```

### 底盘状态监控使用示例
```bash
# 查看电机使能状态
rostopic echo /chassis/motor_enable_status

# 查看故障状态
rostopic echo /chassis/fault_status

# 查看机器人系统状态
rostopic echo /chassis/robot_status

# 查看诊断信息
rostopic echo /chassis/diagnostics
```

底盘状态监控模块会实时处理下位机反馈的状态信息，确保上层应用能够及时了解底盘的运行状态。

## 工具和测试

### 工具目录结构
```
tools/
├── virtual_serial_simulator.py    # 虚拟串口模拟器
├── test_device_switch.py          # 设备开关测试脚本
└── asset/                         # 资源文件目录
```

### 虚拟串口模拟器功能
虚拟串口模拟器(`tools/virtual_serial_simulator.py`)提供以下功能：

1. **自动创建虚拟串口对**：使用socat创建/tmp/ttyV0和/tmp/ttyV1
2. **协议解析**：完整解析上位机17字节控制包
3. **数据包生成**：生成标准的50字节反馈包
4. **设备状态模拟**：模拟车灯、超声波、充电、雷达等设备状态
5. **实时反馈**：根据接收的命令实时更新速度和设备状态
6. **调试输出**：可选择显示所有收发数据包用于调试

### 设备开关测试脚本功能
设备开关测试脚本(`tools/test_device_switch.py`)提供以下测试功能：

1. **自动检测可用话题**：检测当前运行的chassis_controller支持的设备话题
2. **立即更新测试**：测试设备开关状态的立即更新功能
3. **组合测试**：测试多个设备同时开关的功能
4. **速度组合测试**：测试在运动状态下的设备开关功能
5. **急停测试**：测试急停功能在各种状态下的表现
6. **自动重置测试**：测试2秒后开关状态自动重置功能

### 完整测试流程

#### 1. 基础功能测试
```bash
# 终端1：启动虚拟串口模拟器
cd ~/catkin_ws/src/dr100/dr100_chassis_driver
python3 tools/virtual_serial_simulator.py --show-packets

# 终端2：启动底盘控制器
roslaunch dr100_chassis_driver chassis_controller.launch

# 终端3：测试速度控制
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 终端4：查看里程计
rostopic echo /odom
```

#### 2. 设备控制测试
```bash
# 运行自动化设备测试
python3 tools/test_device_switch.py

# 手动测试单个设备
rostopic pub /light_switch std_msgs/Bool "data: true"
rostopic pub /emergency_stop std_msgs/Bool "data: true"
```

#### 3. 状态监控测试
```bash
# 查看底盘状态
rostopic echo /chassis/motor_enable_status
rostopic echo /chassis/fault_status
rostopic echo /chassis/robot_status
rostopic echo /chassis/diagnostics

# 查看电池状态
rostopic echo /battery_state
```

### 资源文件说明

#### BatteryState.md
包含电池状态消息的详细说明和使用示例，描述了sensor_msgs/BatteryState消息的各个字段含义。

#### 示例文件
- **上位机发送数据.csv**：包含上位机发送的17字节控制包的详细格式和示例数据
- **下位机发送的数据.csv**：包含下位机发送的50字节反馈包的详细格式和示例数据
- **里程计模板文件**：提供nav_msgs/Odometry消息的格式模板和完整示例

## 故障排除

### 常见问题及解决方案

#### 1. 串口权限问题
```bash
# 检查串口权限
ls -l /dev/ttyUSB* /tmp/ttyV*

# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 重新登录或重启终端
```

#### 2. 虚拟串口创建失败
```bash
# 检查socat是否安装
which socat

# 安装socat（Ubuntu/Debian）
sudo apt-get install socat

# 手动清理虚拟串口
sudo rm -f /tmp/ttyV0 /tmp/ttyV1
```

#### 3. 节点启动失败
```bash
# 检查ROS环境
echo $ROS_PACKAGE_PATH

# 重新编译
cd ~/catkin_ws
catkin_make

# 重新source环境
source ~/catkin_ws/devel/setup.bash
```

#### 4. 话题无数据
```bash
# 检查话题列表
rostopic list

# 检查话题信息
rostopic info /cmd_vel
rostopic info /odom

# 检查节点状态
rosnode list
rosnode info chassis_controller
```

## 注意事项

1. **权限要求**：确保串口设备有正确的读写权限
2. **测试环境**：虚拟串口模拟器仅用于测试，实际使用时需要连接真实的串口设备
3. **速度限制**：速度命令会被限制在设定的最大值范围内
4. **超时保护**：当命令超时时，节点会自动发送停止命令
5. **自动重连**：串口断开时，节点会自动尝试重连，无需手动重启
6. **频率限制**：
   - 里程计发布频率会自动限制在0.1-1000Hz范围内，设置为≤0时将禁用里程计发布
   - 底盘状态监控发布频率会自动限制在0.1-100Hz范围内，设置为≤0时将禁用状态发布
   - 电池状态发布频率会自动限制在0.1-100Hz范围内
7. **内部状态**：即使禁用里程计发布，内部的里程计积分仍会继续进行，只是不发布消息
8. **模块化设计**：各功能模块可以独立启用/禁用，便于根据实际需求进行配置
9. **调试工具**：使用虚拟串口模拟器的`--show-packets`参数可以查看所有数据包传输，便于调试
10. **测试脚本**：使用提供的测试脚本可以快速验证各项功能是否正常工作
