#include "dr100_chassis_driver/device_control.h"
#include <chrono>

using namespace dr100_chassis_driver;

DeviceControl::DeviceControl()
    : nh_(nullptr)
    , light_switch_(0)
    , ultrasonic_switch_(0)
    , charge_switch_(0)
    , lidar_switch_(0)
    , is_initialized_(false)
    , is_running_(false)
    , shutdown_requested_(false)
    , switch_timeout_(2.0)  // 2秒超时
{
    ROS_DEBUG("DeviceControl constructor");
}

DeviceControl::~DeviceControl()
{
    stop();
    ROS_DEBUG("DeviceControl destructor");
}

bool DeviceControl::initialize(ros::NodeHandle& nh,
                              const std::string& light_topic,
                              const std::string& ultrasonic_topic,
                              const std::string& charge_topic,
                              const std::string& lidar_topic)
{
    if (is_initialized_.load()) {
        ROS_WARN("DeviceControl already initialized");
        return true;
    }

    try {
        nh_ = &nh;

        // 创建话题订阅者
        light_sub_ = nh_->subscribe(light_topic, 1,
                                   &DeviceControl::lightSwitchCallback, this);
        ultrasonic_sub_ = nh_->subscribe(ultrasonic_topic, 1,
                                        &DeviceControl::ultrasonicSwitchCallback, this);
        charge_sub_ = nh_->subscribe(charge_topic, 1,
                                    &DeviceControl::chargeSwitchCallback, this);
        lidar_sub_ = nh_->subscribe(lidar_topic, 1,
                                   &DeviceControl::lidarSwitchCallback, this);

        // 初始化设备状态为默认值（0表示保持状态）
        light_switch_.store(0);
        ultrasonic_switch_.store(0);
        charge_switch_.store(0);
        lidar_switch_.store(0);

        // 初始化时间戳
        const auto now = ros::Time::now();
        last_light_time_ = now;
        last_ultrasonic_time_ = now;
        last_charge_time_ = now;
        last_lidar_time_ = now;

        is_initialized_.store(true);
        
        ROS_INFO("DeviceControl initialized successfully");
        ROS_INFO("  Light switch topic: %s", light_topic.c_str());
        ROS_INFO("  Ultrasonic switch topic: %s", ultrasonic_topic.c_str());
        ROS_INFO("  Charge switch topic: %s", charge_topic.c_str());
        ROS_INFO("  Lidar switch topic: %s", lidar_topic.c_str());
        
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("DeviceControl initialization failed: %s", e.what());
        return false;
    }
}

void DeviceControl::start()
{
    if (!is_initialized_.load()) {
        ROS_ERROR("DeviceControl not initialized");
        return;
    }

    if (is_running_.load()) {
        ROS_WARN("DeviceControl already running");
        return;
    }

    is_running_.store(true);
    shutdown_requested_.store(false);

    // 启动状态重置线程
    reset_thread_ = std::make_unique<std::thread>(&DeviceControl::resetSwitchThread, this);

    ROS_INFO("DeviceControl started");
}

void DeviceControl::stop()
{
    if (!is_running_.load()) {
        return;
    }

    // 请求关闭
    shutdown_requested_.store(true);
    is_running_.store(false);

    // 唤醒重置线程并等待其结束
    {
        std::lock_guard<std::mutex> lock(reset_mutex_);
        reset_cv_.notify_all();
    }

    if (reset_thread_ && reset_thread_->joinable()) {
        reset_thread_->join();
        reset_thread_.reset();
    }

    // 关闭订阅者
    if (light_sub_) light_sub_.shutdown();
    if (ultrasonic_sub_) ultrasonic_sub_.shutdown();
    if (charge_sub_) charge_sub_.shutdown();
    if (lidar_sub_) lidar_sub_.shutdown();

    ROS_INFO("DeviceControl stopped");
}

uint8_t DeviceControl::getLightSwitch() const
{
    return light_switch_.load();
}

uint8_t DeviceControl::getUltrasonicSwitch() const
{
    return ultrasonic_switch_.load();
}

uint8_t DeviceControl::getChargeSwitch() const
{
    return charge_switch_.load();
}

uint8_t DeviceControl::getLidarSwitch() const
{
    return lidar_switch_.load();
}

void DeviceControl::setDeviceStateCallback(const DeviceStateCallback& callback)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    device_state_callback_ = callback;
}

void DeviceControl::lightSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!is_running_.load()) return;

    // 根据新协议：0保持，1开启，2关闭
    const uint8_t new_value = msg->data ? 1 : 2;
    const uint8_t old_value = light_switch_.exchange(new_value);

    // 更新时间戳
    last_light_time_ = ros::Time::now();

    if (new_value != old_value) {
        ROS_DEBUG("Light switch changed: %u -> %u (1=开启, 2=关闭)", old_value, new_value);
        notifyDeviceStateChanged();
    }
}

void DeviceControl::ultrasonicSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!is_running_.load()) return;

    // 0保持，1开启，2关闭
    const uint8_t new_value = msg->data ? 1 : 2;
    const uint8_t old_value = ultrasonic_switch_.exchange(new_value);

    // 更新时间戳
    last_ultrasonic_time_ = ros::Time::now();

    if (new_value != old_value) {
        ROS_DEBUG("Ultrasonic switch changed: %u -> %u (1=开启, 2=关闭)", old_value, new_value);
        notifyDeviceStateChanged();
    }
}

void DeviceControl::chargeSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!is_running_.load()) return;

    // 0保持，1开启，2关闭
    const uint8_t new_value = msg->data ? 1 : 2;
    const uint8_t old_value = charge_switch_.exchange(new_value);

    // 更新时间戳
    last_charge_time_ = ros::Time::now();

    if (new_value != old_value) {
        ROS_DEBUG("Charge switch changed: %u -> %u (1=开启, 2=关闭)", old_value, new_value);
        notifyDeviceStateChanged();
    }
}

void DeviceControl::lidarSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!is_running_.load()) return;

    // 0保持，1开启，2关闭
    const uint8_t new_value = msg->data ? 1 : 2;
    const uint8_t old_value = lidar_switch_.exchange(new_value);

    // 更新时间戳
    last_lidar_time_ = ros::Time::now();

    if (new_value != old_value) {
        ROS_DEBUG("Lidar switch changed: %u -> %u (1=开启, 2=关闭)", old_value, new_value);
        notifyDeviceStateChanged();
    }
}

void DeviceControl::notifyDeviceStateChanged()
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (device_state_callback_) {
        device_state_callback_();
    }
}

void DeviceControl::resetSwitchThread()
{
    ROS_DEBUG("DeviceControl reset thread started");

    while (!shutdown_requested_.load()) {
        std::unique_lock<std::mutex> lock(reset_mutex_);

        // 等待一段时间或直到被唤醒
        reset_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
            return shutdown_requested_.load();
        });

        if (shutdown_requested_.load()) {
            break;
        }

        // 检查并重置超时的开关状态
        const auto now = ros::Time::now();
        resetSwitchIfTimeout(light_switch_, last_light_time_, "light");
        resetSwitchIfTimeout(ultrasonic_switch_, last_ultrasonic_time_, "ultrasonic");
        resetSwitchIfTimeout(charge_switch_, last_charge_time_, "charge");
        resetSwitchIfTimeout(lidar_switch_, last_lidar_time_, "lidar");
    }

    ROS_DEBUG("DeviceControl reset thread stopped");
}

void DeviceControl::resetSwitchIfTimeout(std::atomic<uint8_t>& switch_state,
                                        const ros::Time& last_time,
                                        const char* switch_name)
{
    const auto now = ros::Time::now();
    const auto time_diff = (now - last_time).toSec();

    // 如果超过超时时间且当前状态不是保持状态(0)，则重置为保持状态
    if (time_diff > switch_timeout_ && switch_state.load() != 0) {
        const uint8_t old_value = switch_state.exchange(0);
        if (old_value != 0) {
            ROS_DEBUG("%s switch reset to hold state (0) after %.1fs timeout", switch_name, time_diff);
            notifyDeviceStateChanged();
        }
    }
}
