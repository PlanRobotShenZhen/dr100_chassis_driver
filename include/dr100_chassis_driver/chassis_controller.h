#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <memory>
#include <mutex>
#include <atomic>
#include "dr100_chassis_driver/common_types.h"
#include "dr100_chassis_driver/serial_communication.h"
#include "dr100_chassis_driver/odometry_publisher.h"
#include "dr100_chassis_driver/battery_monitor.h"
#include "dr100_chassis_driver/device_control.h"
#include "dr100_chassis_driver/chassis_status_monitor.h"

class ChassisController
{
public:
    ChassisController();
    ~ChassisController();

    bool initialize();
    void run();
    void shutdown();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cmd_vel_sub_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;

    // 模块组件
    std::unique_ptr<dr100_chassis_driver::SerialCommunication> serial_comm_;
    std::unique_ptr<dr100_chassis_driver::OdometryPublisher> odom_publisher_;
    std::unique_ptr<dr100_chassis_driver::BatteryMonitor> battery_monitor_;
    std::unique_ptr<dr100_chassis_driver::DeviceControl> device_control_;
    std::unique_ptr<dr100_chassis_driver::ChassisStatusMonitor> chassis_status_monitor_;

    // 回调函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void onFeedbackReceived(const dr100_chassis_driver::FeedbackPacket& packet);
    void onSerialError(const char* error_msg);
    void onDeviceStateChanged();

    // 数据处理函数
    dr100_chassis_driver::ControlPacket createControlPacket(double linear_x, double linear_y, double angular_z);

    // 辅助函数
    template<typename T>
    constexpr T clampValue(T value, T max_val) const noexcept {
        return (value > max_val) ? max_val : ((value < -max_val) ? -max_val : value);
    }

    // 参数
    std::string port_name_;
    int baudrate_;
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
    bool odom_publish_tf_;
    double cmd_timeout_;
    std::string battery_topic_;
    double battery_publish_rate_;
    std::string light_topic_;
    std::string ultrasonic_topic_;
    std::string charge_topic_;
    std::string lidar_topic_;
    std::string emergency_topic_;
    std::string motor_enable_topic_;
    std::string chassis_motor_enable_status_topic_;
    std::string chassis_fault_status_topic_;
    std::string chassis_robot_status_topic_;
    std::string chassis_diagnostics_topic_;
    double chassis_status_publish_rate_;

    // 设备启用参数
    bool enable_light_;
    bool enable_ultrasonic_;
    bool enable_charge_;
    bool enable_lidar_;
    bool enable_emergency_;
    bool enable_motor_enable_;
    bool enable_chassis_status_;

    // 状态变量
    std::atomic<bool> is_initialized_;
    std::atomic<bool> shutdown_requested_;
    ros::Time last_cmd_time_;

    // 当前速度状态（用于设备状态变化时重发）
    std::atomic<double> current_linear_x_;
    std::atomic<double> current_linear_y_;
    std::atomic<double> current_angular_z_;

    // 线程同步
    mutable std::mutex cmd_mutex_;
};


#endif // CHASSIS_CONTROLLER_H
