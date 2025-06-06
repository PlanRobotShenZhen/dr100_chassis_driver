#include "dr100_chassis_driver/chassis_status_monitor.h"

namespace dr100_chassis_driver {

ChassisStatusMonitor::ChassisStatusMonitor()
    : nh_(nullptr)
    , publish_rate_(10.0)
    , publish_enabled_(false)
    , is_initialized_(false)
    , is_running_(false)
    , has_new_feedback_(false)
    , current_motor_enable_status_(MotorEnableStatus::DISABLED)
    , current_fault_status_(FaultStatus::NORMAL)
    , current_robot_status_(RobotSystemStatus::STOP_STANDBY)
    , last_motor_enable_status_(MotorEnableStatus::DISABLED)
    , last_fault_status_(FaultStatus::NORMAL)
    , last_robot_status_(RobotSystemStatus::STOP_STANDBY)
{
}

ChassisStatusMonitor::~ChassisStatusMonitor()
{
    stop();
}

bool ChassisStatusMonitor::initialize(ros::NodeHandle& nh, 
                                    const std::string& motor_enable_status_topic,
                                    const std::string& fault_status_topic,
                                    const std::string& robot_status_topic,
                                    const std::string& diagnostics_topic,
                                    double publish_rate)
{
    if (is_initialized_) {
        ROS_WARN("ChassisStatusMonitor already initialized");
        return true;
    }

    nh_ = &nh;
    motor_enable_status_topic_ = motor_enable_status_topic;
    fault_status_topic_ = fault_status_topic;
    robot_status_topic_ = robot_status_topic;
    diagnostics_topic_ = diagnostics_topic;
    publish_rate_ = publish_rate;

    // 检查发布频率设置
    publish_enabled_ = publish_rate_ > 0.0;
    if (publish_enabled_) {
        publish_rate_ = std::max(0.1, std::min(100.0, publish_rate_));
    }

    // 初始化ROS组件
    motor_enable_status_pub_ = nh_->advertise<std_msgs::Bool>(motor_enable_status_topic_, 1);
    fault_status_pub_ = nh_->advertise<std_msgs::UInt8>(fault_status_topic_, 1);
    robot_status_pub_ = nh_->advertise<std_msgs::UInt8>(robot_status_topic_, 1);
    diagnostics_pub_ = nh_->advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 1);

    if (publish_enabled_) {
        status_timer_ = nh_->createTimer(ros::Duration(1.0 / publish_rate_),
                                       &ChassisStatusMonitor::statusTimerCallback, this);
    }

    is_initialized_.store(true);
    ROS_INFO("ChassisStatusMonitor initialized with publish rate: %.1f Hz", publish_rate_);
    return true;
}

void ChassisStatusMonitor::start()
{
    if (!is_initialized_) {
        ROS_ERROR("ChassisStatusMonitor not initialized");
        return;
    }

    is_running_.store(true);
    ROS_DEBUG("ChassisStatusMonitor started");
}

void ChassisStatusMonitor::stop()
{
    is_running_.store(false);
    
    if (status_timer_.isValid()) {
        status_timer_.stop();
    }
    
    ROS_DEBUG("ChassisStatusMonitor stopped");
}

void ChassisStatusMonitor::processFeedbackPacket(const FeedbackPacket& packet)
{
    if (!is_initialized_ || !is_running_) return;

    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_feedback_ = packet;
    has_new_feedback_ = true;

    // 更新状态
    updateStatusFromPacket(packet);

    // 如果不使用定时器发布，则立即发布
    if (!publish_enabled_) {
        publishChassisStatus();
    }
}

void ChassisStatusMonitor::statusTimerCallback(const ros::TimerEvent& event)
{
    if (!is_initialized_ || !is_running_ || !publish_enabled_) return;

    std::lock_guard<std::mutex> lock(status_mutex_);
    if (has_new_feedback_) {
        publishChassisStatus();
        has_new_feedback_ = false;
    }
}

void ChassisStatusMonitor::publishChassisStatus()
{
    const auto current_time = ros::Time::now();

    // 发布电机使能状态
    std_msgs::Bool motor_enable_msg;
    motor_enable_msg.data = (current_motor_enable_status_.load() == MotorEnableStatus::ENABLED);
    motor_enable_status_pub_.publish(motor_enable_msg);

    // 发布故障状态
    std_msgs::UInt8 fault_msg;
    fault_msg.data = static_cast<uint8_t>(current_fault_status_.load());
    fault_status_pub_.publish(fault_msg);

    // 发布机器人系统状态
    std_msgs::UInt8 robot_status_msg;
    robot_status_msg.data = static_cast<uint8_t>(current_robot_status_.load());
    robot_status_pub_.publish(robot_status_msg);

    // 发布诊断信息
    publishDiagnostics();

    // 检查状态变化并记录日志
    const auto motor_status = current_motor_enable_status_.load();
    const auto fault_status = current_fault_status_.load();
    const auto robot_status = current_robot_status_.load();

    if (motor_status != last_motor_enable_status_) {
        ROS_INFO("Motor enable status changed: %s", getMotorEnableStatusString(motor_status).c_str());
        last_motor_enable_status_ = motor_status;
    }

    if (fault_status != last_fault_status_) {
        ROS_INFO("Fault status changed: %s", getFaultStatusString(fault_status).c_str());
        last_fault_status_ = fault_status;
    }

    if (robot_status != last_robot_status_) {
        ROS_INFO("Robot system status changed: %s", getRobotSystemStatusString(robot_status).c_str());
        last_robot_status_ = robot_status;
    }
}

void ChassisStatusMonitor::updateStatusFromPacket(const FeedbackPacket& packet)
{
    // 更新电机使能状态
    current_motor_enable_status_.store(convertMotorEnableStatus(packet.motor_enable));
    
    // 更新故障状态
    current_fault_status_.store(convertFaultStatus(packet.fault_info));
    
    // 更新机器人系统状态
    current_robot_status_.store(convertRobotSystemStatus(packet.robot_status));
}

void ChassisStatusMonitor::publishDiagnostics()
{
    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus diag_status;
    diag_status.name = "chassis_status";
    diag_status.hardware_id = "dr100_chassis";

    const auto fault_status = current_fault_status_.load();
    const auto robot_status = current_robot_status_.load();
    const auto motor_status = current_motor_enable_status_.load();

    // 设置诊断级别
    diag_status.level = getDiagnosticLevel(fault_status, robot_status);

    // 设置诊断消息
    if (fault_status != FaultStatus::NORMAL) {
        diag_status.message = "Chassis fault detected: " + getFaultStatusString(fault_status);
    } else if (robot_status == RobotSystemStatus::EMERGENCY_STOP) {
        diag_status.message = "Emergency stop activated";
    } else if (motor_status == MotorEnableStatus::DISABLED) {
        diag_status.message = "Motors disabled";
    } else {
        diag_status.message = "Chassis operating normally";
    }

    // 添加详细状态信息
    diagnostic_msgs::KeyValue kv;
    
    kv.key = "motor_enable_status";
    kv.value = getMotorEnableStatusString(motor_status);
    diag_status.values.push_back(kv);

    kv.key = "fault_status";
    kv.value = getFaultStatusString(fault_status);
    diag_status.values.push_back(kv);

    kv.key = "robot_system_status";
    kv.value = getRobotSystemStatusString(robot_status);
    diag_status.values.push_back(kv);

    diag_array.status.push_back(diag_status);
    diagnostics_pub_.publish(diag_array);
}

MotorEnableStatus ChassisStatusMonitor::convertMotorEnableStatus(uint8_t raw_status) const
{
    return (raw_status == 1) ? MotorEnableStatus::ENABLED : MotorEnableStatus::DISABLED;
}

FaultStatus ChassisStatusMonitor::convertFaultStatus(uint8_t raw_status) const
{
    switch (raw_status) {
        case 0: return FaultStatus::NORMAL;
        case 1: return FaultStatus::SINGLE_MOTOR_FAULT;
        case 2: return FaultStatus::MULTI_MOTOR_FAULT;
        default: return FaultStatus::NORMAL;
    }
}

RobotSystemStatus ChassisStatusMonitor::convertRobotSystemStatus(uint8_t raw_status) const
{
    switch (raw_status) {
        case 0: return RobotSystemStatus::STOP_STANDBY;
        case 1: return RobotSystemStatus::MOVING;
        case 2: return RobotSystemStatus::FAULT;
        case 3: return RobotSystemStatus::EMERGENCY_STOP;
        default: return RobotSystemStatus::STOP_STANDBY;
    }
}

std::string ChassisStatusMonitor::getMotorEnableStatusString(MotorEnableStatus status) const
{
    switch (status) {
        case MotorEnableStatus::DISABLED: return "DISABLED";
        case MotorEnableStatus::ENABLED: return "ENABLED";
        default: return "UNKNOWN";
    }
}

std::string ChassisStatusMonitor::getFaultStatusString(FaultStatus status) const
{
    switch (status) {
        case FaultStatus::NORMAL: return "NORMAL";
        case FaultStatus::SINGLE_MOTOR_FAULT: return "SINGLE_MOTOR_FAULT";
        case FaultStatus::MULTI_MOTOR_FAULT: return "MULTI_MOTOR_FAULT";
        default: return "UNKNOWN";
    }
}

std::string ChassisStatusMonitor::getRobotSystemStatusString(RobotSystemStatus status) const
{
    switch (status) {
        case RobotSystemStatus::STOP_STANDBY: return "STOP_STANDBY";
        case RobotSystemStatus::MOVING: return "MOVING";
        case RobotSystemStatus::FAULT: return "FAULT";
        case RobotSystemStatus::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

uint8_t ChassisStatusMonitor::getDiagnosticLevel(FaultStatus fault_status, RobotSystemStatus robot_status) const
{
    if (fault_status == FaultStatus::MULTI_MOTOR_FAULT || robot_status == RobotSystemStatus::FAULT) {
        return diagnostic_msgs::DiagnosticStatus::ERROR;
    } else if (fault_status == FaultStatus::SINGLE_MOTOR_FAULT || robot_status == RobotSystemStatus::EMERGENCY_STOP) {
        return diagnostic_msgs::DiagnosticStatus::WARN;
    } else {
        return diagnostic_msgs::DiagnosticStatus::OK;
    }
}

} // namespace dr100_chassis_driver
