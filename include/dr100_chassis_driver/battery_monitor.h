#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <mutex>
#include <atomic>
#include "dr100_chassis_driver/common_types.h"

namespace dr100_chassis_driver {

class BatteryMonitor
{
public:
    BatteryMonitor();
    ~BatteryMonitor();

    // 初始化和控制
    bool initialize(ros::NodeHandle& nh, const std::string& battery_topic = "/battery_state",
                   double publish_rate = 10.0);
    void start();
    void stop();

    // 数据处理
    void processFeedbackPacket(const FeedbackPacket& packet);

    // 状态查询
    bool isInitialized() const { return is_initialized_; }
    bool isRunning() const { return is_running_; }

private:
    // ROS相关
    ros::NodeHandle* nh_;
    ros::Publisher battery_pub_;
    ros::Timer battery_timer_;

    // 配置参数
    std::string battery_topic_;
    double publish_rate_;
    bool publish_enabled_;

    // 状态管理
    std::atomic<bool> is_initialized_;
    std::atomic<bool> is_running_;

    // 数据同步
    std::mutex battery_mutex_;
    FeedbackPacket latest_feedback_;
    bool has_new_feedback_;

    // 电池状态消息
    sensor_msgs::BatteryState battery_msg_;

    // 内部方法
    void batteryTimerCallback(const ros::TimerEvent& event);
    void publishBatteryState();
    void updateBatteryMessage(const FeedbackPacket& packet);
    
    // 数据转换辅助函数
    inline float convertVoltage(uint16_t raw_voltage) const noexcept {
        return static_cast<float>(raw_voltage) * INV_BATTERY_VOLTAGE_SCALE;
    }

    inline float convertCurrent(int16_t raw_current) const noexcept {
        return static_cast<float>(raw_current) * INV_BATTERY_CURRENT_SCALE;
    }

    inline float convertTemperature(int8_t raw_temp) const noexcept {
        return static_cast<float>(raw_temp);
    }

    inline float convertPercentage(uint8_t raw_level) const noexcept {
        return std::max(0.0f, std::min(1.0f, static_cast<float>(raw_level) * INV_BATTERY_PERCENTAGE_SCALE));
    }
};

} // namespace dr100_chassis_driver

#endif // BATTERY_MONITOR_H
