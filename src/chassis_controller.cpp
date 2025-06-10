#include "dr100_chassis_driver/chassis_controller.h"
#include <algorithm>
#include <functional>
#include <numeric>

using namespace dr100_chassis_driver;

ChassisController::ChassisController()
    : private_nh_("~")
    , is_initialized_(false)
    , shutdown_requested_(false)
    , motor_enable_(true)
    , current_linear_x_(0.0)
    , current_linear_y_(0.0)
    , current_angular_z_(0.0)
{
    // 获取参数
    private_nh_.param<std::string>("port", port_name_, "/tmp/ttyV1");
    private_nh_.param("baudrate", baudrate_, 115200);
    private_nh_.param("max_linear_velocity", max_linear_velocity_, 2.0);
    private_nh_.param("max_angular_velocity", max_angular_velocity_, 2.0);
    private_nh_.param("cmd_timeout", cmd_timeout_, 1.0);
    private_nh_.param("motor_enable", motor_enable_, true);
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    private_nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    private_nh_.param("reconnect_interval", reconnect_interval_, 2.0);
    private_nh_.param("max_reconnect_attempts", max_reconnect_attempts_, -1);
    private_nh_.param("odom_publish_rate", odom_publish_rate_, 50.0);
    private_nh_.param("odom_publish_tf", odom_publish_tf_, true);
    private_nh_.param<std::string>("battery_topic", battery_topic_, "/battery_state");
    private_nh_.param("battery_publish_rate", battery_publish_rate_, 10.0);
    private_nh_.param<std::string>("light_topic", light_topic_, "/light_switch");
    private_nh_.param<std::string>("ultrasonic_topic", ultrasonic_topic_, "/ultrasonic_switch");
    private_nh_.param<std::string>("charge_topic", charge_topic_, "/charge_switch");
    private_nh_.param<std::string>("lidar_topic", lidar_topic_, "/lidar_switch");
    private_nh_.param<std::string>("emergency_topic", emergency_topic_, "/emergency_stop");
    private_nh_.param<std::string>("motor_enable_topic", motor_enable_topic_, "/motor_enable");
    private_nh_.param<std::string>("chassis_motor_enable_status_topic", chassis_motor_enable_status_topic_, "/chassis/motor_enable_status");
    private_nh_.param<std::string>("chassis_fault_status_topic", chassis_fault_status_topic_, "/chassis/fault_status");
    private_nh_.param<std::string>("chassis_robot_status_topic", chassis_robot_status_topic_, "/chassis/robot_status");
    private_nh_.param<std::string>("chassis_diagnostics_topic", chassis_diagnostics_topic_, "/chassis/diagnostics");
    private_nh_.param("chassis_status_publish_rate", chassis_status_publish_rate_, 10.0);

    // 设备启用参数
    private_nh_.param("enable_light", enable_light_, true);
    private_nh_.param("enable_ultrasonic", enable_ultrasonic_, true);
    private_nh_.param("enable_charge", enable_charge_, true);
    private_nh_.param("enable_lidar", enable_lidar_, true);
    private_nh_.param("enable_emergency", enable_emergency_, true);
    private_nh_.param("enable_motor_enable", enable_motor_enable_, true);
    private_nh_.param("enable_chassis_status", enable_chassis_status_, true);

    // 调试参数
    private_nh_.param("debug_output_enabled", debug_output_enabled_, false);

    ROS_INFO("ChassisController: port=%s, baudrate=%d, odom_rate=%.1fHz, odom_publish_tf=%s, battery_rate=%.1fHz",
             port_name_.c_str(), baudrate_, odom_publish_rate_, odom_publish_tf_ ? "true" : "false", battery_publish_rate_);
    ROS_INFO("Device control topics: light=%s, ultrasonic=%s, charge=%s, lidar=%s",
             light_topic_.c_str(), ultrasonic_topic_.c_str(), charge_topic_.c_str(), lidar_topic_.c_str());
    ROS_INFO("Safety control topics: emergency=%s, motor_enable=%s",
             emergency_topic_.c_str(), motor_enable_topic_.c_str());
    ROS_INFO("Debug output enabled: %s", debug_output_enabled_ ? "true" : "false");
    ROS_INFO("Note: Program will continue running even if serial port is not available");
}

ChassisController::~ChassisController()
{
    shutdown();
}

bool ChassisController::initialize()
{
    try {
        // 创建模块实例
        serial_comm_ = std::make_unique<SerialCommunication>();
        odom_publisher_ = std::make_unique<OdometryPublisher>();
        battery_monitor_ = std::make_unique<BatteryMonitor>();
        device_control_ = std::make_unique<DeviceControl>();

        // 初始化串口通信模块（总是成功，即使串口不存在）
        serial_comm_->initialize(port_name_, baudrate_, reconnect_interval_, max_reconnect_attempts_);

        // 设置调试输出
        serial_comm_->setDebugOutput(debug_output_enabled_);

        // 设置串口回调函数
        serial_comm_->setFeedbackCallback(
            std::bind(&ChassisController::onFeedbackReceived, this, std::placeholders::_1));
        serial_comm_->setErrorCallback(
            std::bind(&ChassisController::onSerialError, this, std::placeholders::_1));

        // 初始化里程计发布模块
        if (!odom_publisher_->initialize(nh_, odom_topic_, odom_frame_id_, base_frame_id_, odom_publish_rate_, odom_publish_tf_)) {
            ROS_ERROR("Failed to initialize odometry publisher");
            return false;
        }

        // 初始化电池监控模块
        if (!battery_monitor_->initialize(nh_, battery_topic_, battery_publish_rate_)) {
            ROS_ERROR("Failed to initialize battery monitor");
            return false;
        }

        // 初始化设备控制模块
        if (!device_control_->initialize(nh_, light_topic_, ultrasonic_topic_, charge_topic_, lidar_topic_,
                                         emergency_topic_, motor_enable_topic_,
                                         enable_light_, enable_ultrasonic_, enable_charge_, enable_lidar_,
                                         enable_emergency_, enable_motor_enable_)) {
            ROS_ERROR("Failed to initialize device control");
            return false;
        }

        // 设置设备状态更新回调
        device_control_->setDeviceStateCallback(
            std::bind(&ChassisController::onDeviceStateChanged, this));

        // 初始化底盘状态监控模块
        if (enable_chassis_status_) {
            chassis_status_monitor_ = std::make_unique<dr100_chassis_driver::ChassisStatusMonitor>();
            if (!chassis_status_monitor_->initialize(nh_, chassis_motor_enable_status_topic_,
                                                    chassis_fault_status_topic_, chassis_robot_status_topic_,
                                                    chassis_diagnostics_topic_, chassis_status_publish_rate_)) {
                ROS_ERROR("Failed to initialize chassis status monitor");
                return false;
            }
        }

        // 初始化ROS组件
        cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &ChassisController::cmdVelCallback, this);
        spinner_ = std::make_unique<ros::AsyncSpinner>(2);

        // 初始化时间戳
        last_cmd_time_ = ros::Time::now();

        is_initialized_ = true;
        ROS_INFO("ChassisController initialized successfully");
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Initialize failed: %s", e.what());
        return false;
    }
}

void ChassisController::run()
{
    if (!is_initialized_) {
        ROS_ERROR("ChassisController not initialized");
        return;
    }

    spinner_->start();

    // 启动模块
    serial_comm_->start();
    odom_publisher_->start();
    battery_monitor_->start();
    device_control_->start();
    if (chassis_status_monitor_) chassis_status_monitor_->start();

    ROS_INFO("ChassisController started");

    // 主线程：命令超时检查和状态监控
    ros::Rate rate(10);
    static const auto stop_packet = createControlPacket(0.0, 0.0, 0.0);

    // 状态显示计数器
    constexpr int STATUS_DISPLAY_INTERVAL = 300; // 10Hz * 30s = 300
    int status_counter = 0;
    bool last_connected_status = false;

    while (ros::ok() && !shutdown_requested_.load()) {
        const auto now = ros::Time::now();

        // 命令超时检查
        const auto time_since_last_cmd = (now - last_cmd_time_).toSec();
        if (time_since_last_cmd > cmd_timeout_) {
            // 更新当前速度状态为0
            current_linear_x_.store(0.0);
            current_linear_y_.store(0.0);
            current_angular_z_.store(0.0);
            serial_comm_->sendControlPacket(stop_packet);
        }


        const bool current_connected = serial_comm_->isConnected();
        if (current_connected != last_connected_status) {
            ROS_INFO_COND(current_connected, "Serial port connected and ready");
            ROS_WARN_COND(!current_connected, "Serial port disconnected, trying to reconnect...");
            last_connected_status = current_connected;
        }

        // 每30秒显示一次状态（如果未连接）
        if (++status_counter >= STATUS_DISPLAY_INTERVAL) {
            status_counter = 0;
            if (!current_connected) {
                ROS_INFO("Chassis controller running, waiting for serial connection to %s",
                         port_name_.c_str());
            }
        }

        rate.sleep();
    }
}

void ChassisController::shutdown()
{
    // 原子操作检查和设置
    bool expected = false;
    if (!shutdown_requested_.compare_exchange_strong(expected, true)) {
        return; // 已经在关闭过程中
    }

    ROS_INFO("Shutting down ChassisController...");

    // 并行停止模块
    if (serial_comm_) serial_comm_->stop();
    if (odom_publisher_) odom_publisher_->stop();
    if (battery_monitor_) battery_monitor_->stop();
    if (device_control_) device_control_->stop();
    if (chassis_status_monitor_) chassis_status_monitor_->stop();
    if (spinner_) spinner_->stop();

    ROS_INFO("Shutdown complete");
}

void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (!is_initialized_.load() || shutdown_requested_.load()) return;

    // 速度限制
    const auto linear_x = clampValue(msg->linear.x, max_linear_velocity_);
    const auto linear_y = clampValue(msg->linear.y, max_linear_velocity_);
    const auto angular_z = clampValue(msg->angular.z, max_angular_velocity_);

    // 更新当前速度状态
    current_linear_x_.store(linear_x);
    current_linear_y_.store(linear_y);
    current_angular_z_.store(angular_z);

    const auto packet = createControlPacket(linear_x, linear_y, angular_z);

    // 减少锁的持有时间
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (serial_comm_->sendControlPacket(packet)) {
        last_cmd_time_ = ros::Time::now();
    }
}

void ChassisController::onFeedbackReceived(const FeedbackPacket& packet)
{
    if (!is_initialized_.load() || shutdown_requested_.load()) return;

    // 将反馈数据传递给里程计发布模块
    odom_publisher_->processFeedbackPacket(packet);

    // 将反馈数据传递给电池监控模块
    battery_monitor_->processFeedbackPacket(packet);

    // 将反馈数据传递给底盘状态监控模块
    if (chassis_status_monitor_) {
        chassis_status_monitor_->processFeedbackPacket(packet);
    }
}

void ChassisController::onSerialError(const char* error_msg)
{
    ROS_WARN("Serial communication error: %s (will keep trying to reconnect)", error_msg);
}

void ChassisController::onDeviceStateChanged()
{
    if (!is_initialized_.load() || shutdown_requested_.load()) return;

    // 设备状态发生变化时，立即发送更新的控制包
    // 使用当前保存的速度值
    const auto linear_x = current_linear_x_.load();
    const auto linear_y = current_linear_y_.load();
    const auto angular_z = current_angular_z_.load();

    const auto packet = createControlPacket(linear_x, linear_y, angular_z);

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (serial_comm_->sendControlPacket(packet)) {
        ROS_DEBUG("Device state changed, control packet sent with velocity (%.3f, %.3f, %.3f)",
                  linear_x, linear_y, angular_z);
    } else {
        ROS_DEBUG("Device state changed, but failed to send control packet");
    }
}

ControlPacket ChassisController::createControlPacket(double linear_x, double linear_y, double angular_z)
{
    ControlPacket packet{};
    packet.frame_header = FRAME_HEADER;

    // 电机使能和急停状态
    if (device_control_) {
        packet.motor_enable = device_control_->getMotorEnable();
        packet.emergency_stop = device_control_->getEmergencyStop();
    } else {
        packet.motor_enable = motor_enable_ ? 1 : 0;  // 使用参数默认值
        packet.emergency_stop = 0;
    }

    packet.x_velocity = static_cast<int16_t>(linear_x * CONTROL_VEL_SCALE);
    packet.y_velocity = static_cast<int16_t>(linear_y * CONTROL_VEL_SCALE);
    packet.z_velocity = static_cast<int16_t>(angular_z * CONTROL_VEL_SCALE);

    // 设备控制状态
    if (device_control_) {
        packet.light_control = device_control_->getLightSwitch();
        packet.ultrasonic_switch = device_control_->getUltrasonicSwitch();
        packet.charge_switch = device_control_->getChargeSwitch();
        packet.lidar_switch = device_control_->getLidarSwitch();
    } else {
        packet.light_control = 0;
        packet.ultrasonic_switch = 0;
        packet.charge_switch = 0;
        packet.lidar_switch = 0;
    }

    // 校验码计算
    const auto* data = reinterpret_cast<const uint8_t*>(&packet);
    packet.checksum = std::accumulate(data, data + CONTROL_CHECKSUM_LENGTH,
                                     uint8_t{0}, std::bit_xor<uint8_t>());
    packet.frame_tail = FRAME_TAIL;

    return packet;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_controller");

    ChassisController controller;
    if (!controller.initialize()) {
        ROS_ERROR("Initialize failed");
        return -1;
    }

    controller.run();
    return 0;
}
