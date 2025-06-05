## 模块职责划分

SerialCommunication模块：
    串口初始化和重连
    数据包发送和接收
    串口状态管理

OdometryPublisher模块：

    里程计数据处理
    TF变换发布
    里程计话题发布

DeviceControl模块：

    车灯控制话题处理
    超声波开关控制
    充电开关控制
    雷达开关控制

BatteryMonitor模块：

    电池信息话题发布
    电池状态监控
    电池告警处理

EmergencyHandler模块：

    急停命令处理
    安全状态管理
