#ifndef DEVICE_CONTROL_H
#define DEVICE_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <atomic>
#include <functional>
#include <thread>
#include <condition_variable>

namespace dr100_chassis_driver {

class DeviceControl
{
public:
    // 设备状态更新回调函数类型
    using DeviceStateCallback = std::function<void()>;

    DeviceControl();
    ~DeviceControl();

    // 初始化和控制
    bool initialize(ros::NodeHandle& nh,
                   const std::string& light_topic = "/light_switch",
                   const std::string& ultrasonic_topic = "/ultrasonic_switch",
                   const std::string& charge_topic = "/charge_switch",
                   const std::string& lidar_topic = "/lidar_switch",
                   const std::string& emergency_topic = "/emergency_stop",
                   const std::string& motor_enable_topic = "/motor_enable",
                   bool enable_light = true,
                   bool enable_ultrasonic = true,
                   bool enable_charge = true,
                   bool enable_lidar = true,
                   bool enable_emergency = true,
                   bool enable_motor_enable = true);
    void start();
    void stop();

    // 获取当前设备状态
    uint8_t getLightSwitch() const;
    uint8_t getUltrasonicSwitch() const;
    uint8_t getChargeSwitch() const;
    uint8_t getLidarSwitch() const;
    uint8_t getEmergencyStop() const;
    uint8_t getMotorEnable() const;

    // 设置设备状态更新回调
    void setDeviceStateCallback(const DeviceStateCallback& callback);

private:
    // ROS相关
    ros::NodeHandle* nh_;
    ros::Subscriber light_sub_;
    ros::Subscriber ultrasonic_sub_;
    ros::Subscriber charge_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber emergency_sub_;
    ros::Subscriber motor_enable_sub_;

    // 回调函数
    void lightSwitchCallback(const std_msgs::Bool::ConstPtr& msg);
    void ultrasonicSwitchCallback(const std_msgs::Bool::ConstPtr& msg);
    void chargeSwitchCallback(const std_msgs::Bool::ConstPtr& msg);
    void lidarSwitchCallback(const std_msgs::Bool::ConstPtr& msg);
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);
    void motorEnableCallback(const std_msgs::Bool::ConstPtr& msg);

    // 设备状态变量
    std::atomic<uint8_t> light_switch_;
    std::atomic<uint8_t> ultrasonic_switch_;
    std::atomic<uint8_t> charge_switch_;
    std::atomic<uint8_t> lidar_switch_;
    std::atomic<uint8_t> emergency_stop_;
    std::atomic<uint8_t> motor_enable_;

    // 状态管理
    std::atomic<bool> is_initialized_;
    std::atomic<bool> is_running_;
    std::atomic<bool> shutdown_requested_;

    // 设备启用状态
    bool light_enabled_;
    bool ultrasonic_enabled_;
    bool charge_enabled_;
    bool lidar_enabled_;
    bool emergency_enabled_;
    bool motor_enable_enabled_;

    // 状态重置相关
    ros::Time last_light_time_;
    ros::Time last_ultrasonic_time_;
    ros::Time last_charge_time_;
    ros::Time last_lidar_time_;
    ros::Time last_emergency_time_;
    ros::Time last_motor_enable_time_;
    double switch_timeout_;  // 开关状态超时时间（秒）

    // 线程管理
    std::unique_ptr<std::thread> reset_thread_;
    std::condition_variable reset_cv_;
    std::mutex reset_mutex_;

    // 回调函数
    DeviceStateCallback device_state_callback_;

    // 线程同步
    mutable std::mutex callback_mutex_;

    // 辅助函数
    void notifyDeviceStateChanged();
    void resetSwitchThread();
    void resetSwitchIfTimeout(std::atomic<uint8_t>& switch_state, const ros::Time& last_time, const char* switch_name);
};

} // namespace dr100_chassis_driver

#endif // DEVICE_CONTROL_H
