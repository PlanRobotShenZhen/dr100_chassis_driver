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

        // 初始化串口通信模块（总是成功，即使串口不存在）
        serial_comm_->initialize(port_name_, baudrate_, reconnect_interval_, max_reconnect_attempts_);

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

    // 主线程：命令超时检查和状态监控
    ros::Rate rate(10);
    static const auto stop_packet = createControlPacket(0.0, 0.0, 0.0);

    // 状态显示计数器
    int status_counter = 0;
    bool last_connected_status = false;

    while (ros::ok() && !shutdown_requested_.load()) {
        const auto now = ros::Time::now();

        // 命令超时检查
        if ((now - last_cmd_time_).toSec() > cmd_timeout_) {
            serial_comm_->sendControlPacket(stop_packet);
        }

        // 连接状态变化检测
        bool current_connected = serial_comm_->isConnected();
        if (current_connected != last_connected_status) {
            if (current_connected) {
                ROS_INFO("Serial port connected and ready");
            } else {
                ROS_WARN("Serial port disconnected, trying to reconnect...");
            }
            last_connected_status = current_connected;
        }

        // 每30秒显示一次状态（如果未连接）
        if (++status_counter >= 300) { // 10Hz * 30s = 300
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
    if (!is_initialized_.load() || shutdown_requested_.load()) return;

    // 速度限制
    const auto linear_x = clampValue(msg->linear.x, max_linear_velocity_);
    const auto linear_y = clampValue(msg->linear.y, max_linear_velocity_);
    const auto angular_z = clampValue(msg->angular.z, max_angular_velocity_);

    const auto packet = createControlPacket(linear_x, linear_y, angular_z);

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
}

void ChassisController::onSerialError(const char* error_msg)
{
    ROS_WARN("Serial communication error: %s (will keep trying to reconnect)", error_msg);
}

ControlPacket ChassisController::createControlPacket(double linear_x, double linear_y, double angular_z)
{
    ControlPacket packet{};
    packet.frame_header = FRAME_HEADER;
    packet.motor_enable = motor_enable_ ? 1 : 0;
    packet.x_velocity = static_cast<int16_t>(linear_x * CONTROL_VEL_SCALE);
    packet.y_velocity = static_cast<int16_t>(linear_y * CONTROL_VEL_SCALE);
    packet.z_velocity = static_cast<int16_t>(angular_z * CONTROL_VEL_SCALE);

    // 优化校验码计算
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
    uint8_t checksum = 0;
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
