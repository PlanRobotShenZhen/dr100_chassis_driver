#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <cstdint>
#include <cstddef>

namespace dr100_chassis_driver {

// 通讯协议常量
constexpr uint8_t FRAME_HEADER = 0x7B;
constexpr uint8_t FRAME_TAIL = 0x7D;
constexpr double CONTROL_VEL_SCALE = 1000.0;  // 控制包速度放大倍数
constexpr double FEEDBACK_VEL_SCALE = 100.0;  // 反馈包速度放大倍数
constexpr size_t CONTROL_CHECKSUM_LENGTH = 15;
constexpr size_t FEEDBACK_CHECKSUM_LENGTH = 48;

// 数据包大小常量
constexpr size_t CONTROL_PACKET_SIZE = 17;
constexpr size_t FEEDBACK_PACKET_SIZE = 50;

// 上位机发送数据格式（17字节）
struct ControlPacket {
    uint8_t frame_header;      // 0x7B
    uint8_t motor_enable;      // 电机使能
    uint8_t emergency_stop;    // 急停命令
    uint8_t light_control;     // 车灯控制
    int16_t x_velocity;        // X轴速度 (放大1000倍)
    int16_t y_velocity;        // Y轴速度 (放大1000倍)
    int16_t z_velocity;        // Z轴速度 (放大1000倍)
    uint8_t ultrasonic_switch; // 超声波开关
    uint8_t charge_switch;     // 充电开关
    uint8_t lidar_switch;      // 雷达开关
    uint8_t reserved1;         // 预留
    uint8_t reserved2;         // 预留
    uint8_t checksum;          // 校验码
    uint8_t frame_tail;        // 0x7D
} __attribute__((packed));

// 下位机发送数据格式（50字节）
struct FeedbackPacket {
    uint8_t frame_header;      // 0x7B
    uint8_t motor_enable;      // 电机使能状态
    uint8_t fault_info;        // 故障信息
    uint8_t robot_status;      // 机器人系统状态
    int16_t x_velocity;        // X轴速度反馈 (放大100倍)
    int16_t y_velocity;        // Y轴速度反馈 (放大100倍)
    int16_t z_velocity;        // Z轴速度反馈 (放大100倍)
    uint16_t battery_voltage;  // 电池电压 (放大100倍)
    int16_t battery_current;   // 电池电流 (放大100倍)
    uint8_t battery_level;     // 电池电量
    int8_t battery_temp;       // 电池温度
    uint8_t motor_status[4];   // 4个电机状态
    uint16_t motor_faults[4];  // 4个电机故障信息
    uint32_t motor_pulses[4];  // 4个电机脉冲数
    uint32_t odometry;         // 里程计 (放大100倍)
    uint8_t checksum;          // 校验码
    uint8_t frame_tail;        // 0x7D
} __attribute__((packed));

// 里程计协方差矩阵
extern const double odom_pose_covariance[36];
extern const double odom_twist_covariance[36];
extern const double odom_pose_covariance2[36];  // 静止时使用
extern const double odom_twist_covariance2[36]; // 静止时使用

} // namespace dr100_chassis_driver

#endif // COMMON_TYPES_H
