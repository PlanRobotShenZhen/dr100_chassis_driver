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

    // 回调函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void onFeedbackReceived(const dr100_chassis_driver::FeedbackPacket& packet);
    void onSerialError(const std::string& error_msg);

    // 数据处理函数
    dr100_chassis_driver::ControlPacket createControlPacket(double linear_x, double linear_y, double angular_z);

    // 辅助函数
    template<typename T>
    T clampValue(T value, T max_val) const {
        return std::max(-max_val, std::min(max_val, value));
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
    double cmd_timeout_;

    // 状态变量
    std::atomic<bool> is_initialized_;
    std::atomic<bool> shutdown_requested_;
    ros::Time last_cmd_time_;

    // 线程同步
    mutable std::mutex cmd_mutex_;
};


#endif // CHASSIS_CONTROLLER_H
