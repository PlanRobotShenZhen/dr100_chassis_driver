#include "dr100_chassis_driver/chassis_controller.h"
#include <algorithm>
#include <functional>

using namespace dr100_chassis_driver;

ChassisController::ChassisController()
    : private_nh_("~")
    , is_initialized_(false)
    , shutdown_requested_(false)
    , motor_enable_(true)
{
    // 获取参数
    private_nh_.param("port", port_name_, std::string("/tmp/ttyV1"));
    private_nh_.param("baudrate", baudrate_, 115200);
    private_nh_.param("max_linear_velocity", max_linear_velocity_, 2.0);
    private_nh_.param("max_angular_velocity", max_angular_velocity_, 2.0);
    private_nh_.param("cmd_timeout", cmd_timeout_, 1.0);
    private_nh_.param("motor_enable", motor_enable_, true);
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("reconnect_interval", reconnect_interval_, 2.0);
    private_nh_.param("max_reconnect_attempts", max_reconnect_attempts_, -1);
    private_nh_.param("odom_publish_rate", odom_publish_rate_, 50.0);

    ROS_INFO("ChassisController: port=%s, baudrate=%d, odom_rate=%.1fHz",
             port_name_.c_str(), baudrate_, odom_publish_rate_);
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

        // 初始化串口通信模块
        if (!serial_comm_->initialize(port_name_, baudrate_, reconnect_interval_, max_reconnect_attempts_)) {
            ROS_ERROR("Failed to initialize serial communication");
            return false;
        }

        // 设置串口回调函数
        serial_comm_->setFeedbackCallback(
            std::bind(&ChassisController::onFeedbackReceived, this, std::placeholders::_1));
        serial_comm_->setErrorCallback(
            std::bind(&ChassisController::onSerialError, this, std::placeholders::_1));

        // 初始化里程计发布模块
        if (!odom_publisher_->initialize(nh_, odom_topic_, odom_frame_id_, base_frame_id_, odom_publish_rate_)) {
            ROS_ERROR("Failed to initialize odometry publisher");
            return false;
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

    ROS_INFO("ChassisController started");

    // 主线程：命令超时检查
    ros::Rate rate(10);
    while (ros::ok() && !shutdown_requested_) {
        auto now = ros::Time::now();
        if ((now - last_cmd_time_).toSec() > cmd_timeout_) {
            static auto stop_packet = createControlPacket(0.0, 0.0, 0.0);
            serial_comm_->sendControlPacket(stop_packet);
        }
        rate.sleep();
    }
}

void ChassisController::shutdown()
{
    if (shutdown_requested_) return;

    ROS_INFO("Shutting down ChassisController...");
    shutdown_requested_ = true;

    // 停止模块
    if (serial_comm_) serial_comm_->stop();
    if (odom_publisher_) odom_publisher_->stop();

    if (spinner_) spinner_->stop();

    ROS_INFO("Shutdown complete");
}

void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (!is_initialized_ || shutdown_requested_) return;

    // 速度限制
    auto linear_x = clampValue(msg->linear.x, max_linear_velocity_);
    auto linear_y = clampValue(msg->linear.y, max_linear_velocity_);
    auto angular_z = clampValue(msg->angular.z, max_angular_velocity_);

    auto packet = createControlPacket(linear_x, linear_y, angular_z);

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (serial_comm_->sendControlPacket(packet)) {
        last_cmd_time_ = ros::Time::now();
        ROS_DEBUG("Sent cmd: [%.2f, %.2f, %.2f]", linear_x, linear_y, angular_z);
    }
}

void ChassisController::onFeedbackReceived(const FeedbackPacket& packet)
{
    if (!is_initialized_ || shutdown_requested_) return;

    // 将反馈数据传递给里程计发布模块
    odom_publisher_->processFeedbackPacket(packet);
}

void ChassisController::onSerialError(const std::string& error_msg)
{
    ROS_ERROR("Serial communication error: %s", error_msg.c_str());
}

ControlPacket ChassisController::createControlPacket(double linear_x, double linear_y, double angular_z)
{
    ControlPacket packet{};
    packet.frame_header = FRAME_HEADER;
    packet.motor_enable = motor_enable_ ? 1 : 0;
    packet.x_velocity = static_cast<int16_t>(linear_x * CONTROL_VEL_SCALE);
    packet.y_velocity = static_cast<int16_t>(linear_y * CONTROL_VEL_SCALE);
    packet.z_velocity = static_cast<int16_t>(angular_z * CONTROL_VEL_SCALE);

    // 计算校验码
    uint8_t checksum = 0;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
    for (size_t i = 0; i < CONTROL_CHECKSUM_LENGTH; ++i) {
        checksum ^= data[i];
    }
    packet.checksum = checksum;
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
