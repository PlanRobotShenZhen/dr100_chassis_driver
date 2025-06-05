#include "dr100_chassis_driver/battery_monitor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace dr100_chassis_driver {

BatteryMonitor::BatteryMonitor()
    : nh_(nullptr)
    , is_initialized_(false)
    , is_running_(false)
    , publish_enabled_(false)
    , has_new_feedback_(false)
{
}

BatteryMonitor::~BatteryMonitor()
{
    stop();
}

bool BatteryMonitor::initialize(ros::NodeHandle& nh, const std::string& battery_topic, double publish_rate)
{
    if (is_initialized_) {
        ROS_WARN("BatteryMonitor already initialized");
        return true;
    }

    nh_ = &nh;
    battery_topic_ = battery_topic;
    publish_rate_ = publish_rate;

    // 检查发布频率设置
    publish_enabled_ = publish_rate_ > 0.0;
    if (publish_enabled_) {
        publish_rate_ = std::max(0.1, std::min(100.0, publish_rate_));
    }

    // 初始化ROS组件
    battery_pub_ = nh_->advertise<sensor_msgs::BatteryState>(battery_topic_, 1);

    if (publish_enabled_) {
        battery_timer_ = nh_->createTimer(ros::Duration(1.0 / publish_rate_),
                                        &BatteryMonitor::batteryTimerCallback, this);
    }

    // 初始化电池状态消息
    static constexpr char FRAME_ID[] = "base_link";
    static constexpr char LOCATION[] = "chassis";
    static constexpr char SERIAL_NUMBER[] = "DR100_BATTERY_001";

    battery_msg_.header.frame_id = FRAME_ID;
    battery_msg_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_msg_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_msg_.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_msg_.present = true;
    battery_msg_.location = LOCATION;
    battery_msg_.serial_number = SERIAL_NUMBER;

    // 设置未知值为NaN
    battery_msg_.charge = std::numeric_limits<float>::quiet_NaN();
    battery_msg_.capacity = std::numeric_limits<float>::quiet_NaN();
    battery_msg_.design_capacity = std::numeric_limits<float>::quiet_NaN();

    is_initialized_ = true;
    ROS_INFO("BatteryMonitor initialized: topic=%s, rate=%.1fHz", 
             battery_topic_.c_str(), publish_enabled_ ? publish_rate_ : 0.0);
    return true;
}

void BatteryMonitor::start()
{
    if (!is_initialized_) {
        ROS_ERROR("BatteryMonitor not initialized");
        return;
    }

    if (is_running_) {
        ROS_WARN("BatteryMonitor already running");
        return;
    }

    is_running_ = true;
    ROS_INFO("BatteryMonitor started");
}

void BatteryMonitor::stop()
{
    if (!is_running_) return;

    is_running_ = false;
    
    if (battery_timer_.isValid()) {
        battery_timer_.stop();
    }

    ROS_INFO("BatteryMonitor stopped");
}

void BatteryMonitor::processFeedbackPacket(const FeedbackPacket& packet)
{
    if (!is_initialized_.load() || !is_running_.load()) return;

    std::lock_guard<std::mutex> lock(battery_mutex_);
    latest_feedback_ = packet;
    has_new_feedback_ = true;

    // 如果不使用定时器发布，则立即发布
    if (!publish_enabled_) {
        publishBatteryState();
    }
}

void BatteryMonitor::batteryTimerCallback(const ros::TimerEvent& event)
{
    if (!is_initialized_.load() || !is_running_.load() || !publish_enabled_) return;

    std::lock_guard<std::mutex> lock(battery_mutex_);
    if (has_new_feedback_) {
        publishBatteryState();
        has_new_feedback_ = false;
    }
}

void BatteryMonitor::publishBatteryState()
{
    updateBatteryMessage(latest_feedback_);
    battery_msg_.header.stamp = ros::Time::now();
    battery_pub_.publish(battery_msg_);
}

void BatteryMonitor::updateBatteryMessage(const FeedbackPacket& packet)
{
    // 转换电池数据
    battery_msg_.voltage = convertVoltage(packet.battery_voltage);
    battery_msg_.current = convertCurrent(packet.battery_current);
    battery_msg_.percentage = convertPercentage(packet.battery_level);

    // 温度转换
    const float temp_celsius = convertTemperature(packet.battery_temp);

    // 根据电流方向判断充电状态
    const float abs_current = std::abs(battery_msg_.current);
    if (abs_current < BATTERY_CURRENT_THRESHOLD) {
        battery_msg_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else if (battery_msg_.current > BATTERY_CURRENT_THRESHOLD) {
        battery_msg_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else {
        battery_msg_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }

    // 根据电量百分比判断是否充满
    if (battery_msg_.percentage > BATTERY_FULL_THRESHOLD) {
        battery_msg_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
    }

    // 根据电压和温度判断电池健康状态
    if (battery_msg_.voltage < BATTERY_MIN_VOLTAGE) {
        battery_msg_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else if (battery_msg_.voltage > BATTERY_MAX_VOLTAGE) {
        battery_msg_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    } else if (temp_celsius > BATTERY_MAX_TEMP) {
        battery_msg_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    } else if (temp_celsius < BATTERY_MIN_TEMP) {
        battery_msg_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_COLD;
    } else {
        battery_msg_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
}


} // namespace dr100_chassis_driver
