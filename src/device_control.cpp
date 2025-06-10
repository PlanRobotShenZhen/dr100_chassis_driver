#include "dr100_chassis_driver/device_control.h"
#include <chrono>

using namespace dr100_chassis_driver;

DeviceControl::DeviceControl()
    : nh_(nullptr)
    , light_switch_(0)
    , ultrasonic_switch_(0)
    , charge_switch_(0)
    , lidar_switch_(0)
    , emergency_stop_(0)
    , motor_enable_(0)
    , light_enabled_(false)
    , ultrasonic_enabled_(false)
    , charge_enabled_(false)
    , lidar_enabled_(false)
    , emergency_enabled_(false)
    , motor_enable_enabled_(false)
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
                              const std::string& lidar_topic,
                              const std::string& emergency_topic,
                              const std::string& motor_enable_topic,
                              bool enable_light,
                              bool enable_ultrasonic,
                              bool enable_charge,
                              bool enable_lidar,
                              bool enable_emergency,
                              bool enable_motor_enable)
{
    if (is_initialized_.load()) {
        ROS_WARN("DeviceControl already initialized");
        return true;
    }

    try {
        nh_ = &nh;

        // 保存启用状态
        light_enabled_ = enable_light;
        ultrasonic_enabled_ = enable_ultrasonic;
        charge_enabled_ = enable_charge;
        lidar_enabled_ = enable_lidar;
        emergency_enabled_ = enable_emergency;
        motor_enable_enabled_ = enable_motor_enable;

        // 根据启用状态创建话题订阅者
        if (light_enabled_) {
            light_sub_ = nh_->subscribe(light_topic, 1,
                                       &DeviceControl::lightSwitchCallback, this);
        }
        if (ultrasonic_enabled_) {
            ultrasonic_sub_ = nh_->subscribe(ultrasonic_topic, 1,
                                            &DeviceControl::ultrasonicSwitchCallback, this);
        }
        if (charge_enabled_) {
            charge_sub_ = nh_->subscribe(charge_topic, 1,
                                        &DeviceControl::chargeSwitchCallback, this);
        }
        if (lidar_enabled_) {
            lidar_sub_ = nh_->subscribe(lidar_topic, 1,
                                       &DeviceControl::lidarSwitchCallback, this);
        }
        if (emergency_enabled_) {
            emergency_sub_ = nh_->subscribe(emergency_topic, 1,
                                           &DeviceControl::emergencyStopCallback, this);
        }
        if (motor_enable_enabled_) {
            motor_enable_sub_ = nh_->subscribe(motor_enable_topic, 1,
                                              &DeviceControl::motorEnableCallback, this);
        }

        // 初始化设备状态为默认值（0表示保持状态，电机使能默认开启）
        light_switch_.store(0);
        ultrasonic_switch_.store(0);
        charge_switch_.store(0);
        lidar_switch_.store(0);
        emergency_stop_.store(0);
        motor_enable_.store(1);

        // 初始化时间戳
        const auto now = ros::Time::now();
        last_light_time_ = now;
        last_ultrasonic_time_ = now;
        last_charge_time_ = now;
        last_lidar_time_ = now;
        last_emergency_time_ = now;
        last_motor_enable_time_ = now;

        is_initialized_.store(true);
        
        ROS_INFO("DeviceControl initialized successfully");
        if (light_enabled_) ROS_INFO("  Light switch topic: %s", light_topic.c_str());
        if (ultrasonic_enabled_) ROS_INFO("  Ultrasonic switch topic: %s", ultrasonic_topic.c_str());
        if (charge_enabled_) ROS_INFO("  Charge switch topic: %s", charge_topic.c_str());
        if (lidar_enabled_) ROS_INFO("  Lidar switch topic: %s", lidar_topic.c_str());
        if (emergency_enabled_) ROS_INFO("  Emergency stop topic: %s", emergency_topic.c_str());
        if (motor_enable_enabled_) ROS_INFO("  Motor enable topic: %s", motor_enable_topic.c_str());

        // 显示禁用的设备
        std::vector<std::string> disabled_devices;
        if (!light_enabled_) disabled_devices.push_back("light");
        if (!ultrasonic_enabled_) disabled_devices.push_back("ultrasonic");
        if (!charge_enabled_) disabled_devices.push_back("charge");
        if (!lidar_enabled_) disabled_devices.push_back("lidar");
        if (!emergency_enabled_) disabled_devices.push_back("emergency");
        if (!motor_enable_enabled_) disabled_devices.push_back("motor_enable");

        if (!disabled_devices.empty()) {
            std::string disabled_list;
            for (size_t i = 0; i < disabled_devices.size(); ++i) {
                if (i > 0) disabled_list += ", ";
                disabled_list += disabled_devices[i];
            }
            ROS_INFO("  Disabled devices: %s", disabled_list.c_str());
        }
        
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
    if (emergency_sub_) emergency_sub_.shutdown();
    if (motor_enable_sub_) motor_enable_sub_.shutdown();

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

uint8_t DeviceControl::getEmergencyStop() const
{
    return emergency_stop_.load();
}

uint8_t DeviceControl::getMotorEnable() const
{
    return motor_enable_.load();
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

void DeviceControl::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!is_running_.load()) return;

    // 急停协议：0保持，1急停，2取消急停
    const uint8_t new_value = msg->data ? 1 : 2;
    const uint8_t old_value = emergency_stop_.exchange(new_value);

    // 更新时间戳
    last_emergency_time_ = ros::Time::now();

    if (new_value != old_value) {
        ROS_DEBUG("Emergency stop changed: %u -> %u (1=急停, 2=取消急停)", old_value, new_value);
        notifyDeviceStateChanged();
    }
}

void DeviceControl::motorEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!is_running_.load()) return;

    // 电机使能协议：0保持，1开启，2关闭
    const uint8_t new_value = msg->data ? 1 : 2;
    const uint8_t old_value = motor_enable_.exchange(new_value);

    // 更新时间戳
    last_motor_enable_time_ = ros::Time::now();

    if (new_value != old_value) {
        ROS_DEBUG("Motor enable changed: %u -> %u (1=开启, 2=关闭)", old_value, new_value);
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

        // 检查并重置超时的开关状态（只检查启用的设备）
        const auto now = ros::Time::now();
        if (light_enabled_) resetSwitchIfTimeout(light_switch_, last_light_time_, "light");
        if (ultrasonic_enabled_) resetSwitchIfTimeout(ultrasonic_switch_, last_ultrasonic_time_, "ultrasonic");
        if (charge_enabled_) resetSwitchIfTimeout(charge_switch_, last_charge_time_, "charge");
        if (lidar_enabled_) resetSwitchIfTimeout(lidar_switch_, last_lidar_time_, "lidar");
        if (emergency_enabled_) resetSwitchIfTimeout(emergency_stop_, last_emergency_time_, "emergency_stop");
        // if (motor_enable_enabled_) resetSwitchIfTimeout(motor_enable_, last_motor_enable_time_, "motor_enable");
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
