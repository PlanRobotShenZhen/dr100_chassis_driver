#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <serial/serial.h>
#include <vector>
#include <cstdint>

class ChassisController
{
public:
    ChassisController();
    ~ChassisController();
    
    bool initialize();
    void run();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;
    
    // 串口相关
    serial::Serial serial_port_;
    std::string port_name_;
    int baudrate_;
    
    // 数据包结构
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
    
    // 回调函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // 串口通讯函数
    bool sendControlPacket(const ControlPacket& packet);
    bool receiveFeedbackPacket(FeedbackPacket& packet);
    void processSerialData();

    // 串口重连函数
    bool initializeSerial();
    bool reconnectSerial();
    bool isSerialConnected();
    
    // 数据处理函数
    uint8_t calculateChecksum(const uint8_t* data, size_t length);
    ControlPacket createControlPacket(double linear_x, double linear_y, double angular_z);
    void publishOdometry(const FeedbackPacket& packet);
    void updateOdometry(double vx, double vy, double vth, double dt);
    
    // 参数
    double max_linear_velocity_;
    double max_angular_velocity_;
    bool motor_enable_;
    std::string cmd_vel_topic_;
    std::string odom_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    double reconnect_interval_;
    int max_reconnect_attempts_;
    double odom_publish_rate_;
    bool publish_odom_;

    // 状态变量
    bool is_initialized_;
    ros::Time last_cmd_time_;
    ros::Time last_odom_time_;
    ros::Time last_odom_publish_time_;
    double cmd_timeout_;
    ros::Time last_reconnect_attempt_;
    int reconnect_attempts_;
    bool serial_connected_;

    // 里程计状态
    double x_;
    double y_;
    double th_;
    double vx_;
    double vy_;
    double vth_;
};


// 运动时使用的协方差矩阵
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
// 静止时使用的协方差矩阵
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};

#endif // CHASSIS_CONTROLLER_H
