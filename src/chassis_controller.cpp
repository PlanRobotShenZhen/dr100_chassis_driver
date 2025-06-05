#include "dr100_chassis_driver/chassis_controller.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <functional>
#include <iterator>

ChassisController::ChassisController()
    : private_nh_("~")
    , is_initialized_(false)
    , shutdown_requested_(false)
    , motor_enable_(true)
    , reconnect_attempts_(0)
    , serial_connected_(false)
    , x_(0.0)
    , y_(0.0)
    , th_(0.0)
    , vx_(0.0)
    , vy_(0.0)
    , vth_(0.0)
    , has_new_feedback_(false)
{
    // 获取参数（使用结构化初始化简化）
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

    // 简化里程计发布设置
    publish_odom_ = odom_publish_rate_ > 0.0;
    if (publish_odom_) {
        odom_publish_rate_ = std::max(0.1, std::min(1000.0, odom_publish_rate_));
    }

    // 简化日志输出
    ROS_INFO("ChassisController: port=%s, baudrate=%d, odom_rate=%.1fHz",
             port_name_.c_str(), baudrate_, publish_odom_ ? odom_publish_rate_ : 0.0);
}

ChassisController::~ChassisController()
{
    shutdown();
}

bool ChassisController::initialize()
{
    try {
        if (!initializeSerial()) return false;

        // 初始化ROS组件
        cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &ChassisController::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 1);

        if (publish_odom_) {
            odom_timer_ = nh_.createTimer(ros::Duration(1.0 / odom_publish_rate_),
                                        &ChassisController::odomTimerCallback, this);
        }

        spinner_ = std::make_unique<ros::AsyncSpinner>(2); // 减少线程数

        // 初始化时间戳
        auto now = ros::Time::now();
        last_cmd_time_ = last_odom_time_ = last_reconnect_attempt_ = now;

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

    // 启动工作线程
    serial_thread_ = std::thread([this] { serialProcessingThread(); });
    reconnect_thread_ = std::thread([this] { reconnectionThread(); });

    ROS_INFO("ChassisController started");

    // 主线程：命令超时检查
    ros::Rate rate(10);
    while (ros::ok() && !shutdown_requested_) {
        // 缓存时间计算避免重复调用
        auto now = ros::Time::now();
        if ((now - last_cmd_time_).toSec() > cmd_timeout_) {
            static auto stop_packet = createControlPacket(0.0, 0.0, 0.0); // 缓存停止包
            sendControlPacket(stop_packet);
        }
        rate.sleep();
    }
}

void ChassisController::shutdown()
{
    if (shutdown_requested_) return; // 避免重复关闭

    ROS_INFO("Shutting down ChassisController...");
    shutdown_requested_ = true;
    reconnect_cv_.notify_all();

    // 等待线程结束
    for (auto& thread : {std::ref(serial_thread_), std::ref(reconnect_thread_)}) {
        if (thread.get().joinable()) thread.get().join();
    }

    if (spinner_) spinner_->stop();

    // 关闭串口
    try {
        if (serial_port_.isOpen()) serial_port_.close();
    } catch (const std::exception& e) {
        ROS_WARN("Error closing serial: %s", e.what());
    }

    ROS_INFO("Shutdown complete");
}

void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (!is_initialized_ || shutdown_requested_) return;

    // 使用模板函数简化速度限制
    auto linear_x = clampValue(msg->linear.x, max_linear_velocity_);
    auto linear_y = clampValue(msg->linear.y, max_linear_velocity_);
    auto angular_z = clampValue(msg->angular.z, max_angular_velocity_);

    auto packet = createControlPacket(linear_x, linear_y, angular_z);

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (sendControlPacket(packet)) {
        last_cmd_time_ = ros::Time::now();
        ROS_DEBUG("Sent cmd: [%.2f, %.2f, %.2f]", linear_x, linear_y, angular_z);
    }
}

void ChassisController::odomTimerCallback(const ros::TimerEvent& event)
{
    if (!is_initialized_ || shutdown_requested_ || !publish_odom_) return;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (has_new_feedback_) {
        publishOdometry(latest_feedback_);
        has_new_feedback_ = false;
    }
}

void ChassisController::serialProcessingThread()
{
    ROS_INFO("Serial thread started");
    ros::Rate rate(100);

    while (ros::ok() && !shutdown_requested_) {
        if (isSerialConnected()) processSerialData();
        rate.sleep();
    }

    ROS_INFO("Serial thread exiting");
}

void ChassisController::reconnectionThread()
{
    ROS_INFO("Reconnect thread started");

    while (ros::ok() && !shutdown_requested_) {
        std::unique_lock<std::mutex> lock(serial_mutex_);

        auto timeout = std::chrono::seconds(static_cast<int>(reconnect_interval_));
        if (reconnect_cv_.wait_for(lock, timeout, [this] {
            return shutdown_requested_ || !isSerialConnected();
        })) {
            if (shutdown_requested_) break;

            if (!isSerialConnected() && reconnectSerial()) {
                ROS_INFO("Serial reconnected");
                reconnect_attempts_ = 0;
            }
        }
    }

    ROS_INFO("Reconnect thread exiting");
}

ChassisController::ControlPacket ChassisController::createControlPacket(
    double linear_x, double linear_y, double angular_z)
{
    // 使用常量避免重复计算
    constexpr double VEL_SCALE = 1000.0;
    constexpr uint8_t FRAME_HEADER = 0x7B;
    constexpr uint8_t FRAME_TAIL = 0x7D;
    constexpr size_t CHECKSUM_LENGTH = 15;

    ControlPacket packet{};
    packet.frame_header = FRAME_HEADER;
    packet.motor_enable = motor_enable_ ? 1 : 0;
    packet.x_velocity = static_cast<int16_t>(linear_x * VEL_SCALE);
    packet.y_velocity = static_cast<int16_t>(linear_y * VEL_SCALE);
    packet.z_velocity = static_cast<int16_t>(angular_z * VEL_SCALE);
    packet.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&packet), CHECKSUM_LENGTH);
    packet.frame_tail = FRAME_TAIL;

    return packet;
}

uint8_t ChassisController::calculateChecksum(const uint8_t* data, size_t length)
{
    return std::accumulate(data, data + length, uint8_t{0}, std::bit_xor<uint8_t>());
}

bool ChassisController::sendControlPacket(const ControlPacket& packet)
{
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (!isSerialConnected()) return false;

        auto bytes_written = serial_port_.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));

        if (bytes_written != sizeof(packet)) {
            handleSerialError();
            return false;
        }
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Send packet failed: %s", e.what());
        handleSerialError();
        return false;
    }
}

void ChassisController::processSerialData()
{
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (!isSerialConnected() || serial_port_.available() < sizeof(FeedbackPacket)) return;

        FeedbackPacket packet;
        if (receiveFeedbackPacket(packet)) {
            {
                std::lock_guard<std::mutex> odom_lock(odom_mutex_);
                latest_feedback_ = packet;
                has_new_feedback_ = true;
            }

            if (!publish_odom_) publishOdometry(packet);
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Process serial failed: %s", e.what());
        handleSerialError();
    }
}

bool ChassisController::receiveFeedbackPacket(FeedbackPacket& packet)
{
    try {
        // 使用常量避免重复计算
        constexpr uint8_t FRAME_HEADER = 0x7B;
        constexpr uint8_t FRAME_TAIL = 0x7D;
        constexpr size_t REMAINING_BYTES = sizeof(FeedbackPacket) - 1;
        constexpr size_t CHECKSUM_LENGTH = sizeof(FeedbackPacket) - 2;

        // 寻找帧头
        uint8_t byte;
        while (serial_port_.available() > 0) {
            serial_port_.read(&byte, 1);
            if (byte == FRAME_HEADER) {
                packet.frame_header = byte;
                break;
            }
        }

        if (byte != FRAME_HEADER || serial_port_.available() < REMAINING_BYTES) {
            return false;
        }

        // 读取剩余数据
        auto packet_ptr = reinterpret_cast<uint8_t*>(&packet) + 1;
        auto bytes_read = serial_port_.read(packet_ptr, REMAINING_BYTES);

        // 批量验证
        bool valid_packet = (bytes_read == REMAINING_BYTES) &&
                           (packet.frame_tail == FRAME_TAIL) &&
                           (packet.checksum == calculateChecksum(
                               reinterpret_cast<const uint8_t*>(&packet), CHECKSUM_LENGTH));

        if (!valid_packet) {
            ROS_WARN("Invalid packet: bytes=%zu, tail=0x%02X, checksum=0x%02X",
                     bytes_read, packet.frame_tail, packet.checksum);
        }

        return valid_packet;

    } catch (const std::exception& e) {
        ROS_ERROR("Receive packet failed: %s", e.what());
        serial_connected_ = false;
        return false;
    }
}

void ChassisController::publishOdometry(const FeedbackPacket& packet)
{
    // 避免重复的时间计算
    auto current_time = ros::Time::now();
    auto dt = (current_time - last_odom_time_).toSec();
    last_odom_time_ = current_time;

    // 使用常量避免重复计算
    constexpr double VEL_SCALE = 1.0 / 100.0;

    // 批量转换速度数据
    auto vx = static_cast<double>(packet.x_velocity) * VEL_SCALE;
    auto vy = static_cast<double>(packet.y_velocity) * VEL_SCALE;
    auto vth = static_cast<double>(packet.z_velocity) * VEL_SCALE;

    updateOdometry(vx, vy, vth, dt);

    // 避免重复计算四元数和头部信息
    auto odom_quat = tf::createQuaternionMsgFromYaw(th_);

    // 创建通用头部
    auto createHeader = [&](const std::string& frame_id, const std::string& child_frame = "") {
        std_msgs::Header header;
        header.stamp = current_time;
        header.frame_id = frame_id;
        return header;
    };

    // 发布TF变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = createHeader(odom_frame_id_);
    odom_trans.child_frame_id = base_frame_id_;
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry odom;
    odom.header = createHeader(odom_frame_id_);
    odom.child_frame_id = base_frame_id_;

    // 位置和方向
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // 速度
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;

    // 简化协方差矩阵设置：避免重复循环
    bool is_stationary = (vx_ == 0 && vth_ == 0);
    auto& pose_cov = is_stationary ? odom_pose_covariance2 : odom_pose_covariance;
    auto& twist_cov = is_stationary ? odom_twist_covariance2 : odom_twist_covariance;

    std::copy(pose_cov, pose_cov + 36, odom.pose.covariance.begin());
    std::copy(twist_cov, twist_cov + 36, odom.twist.covariance.begin());

    odom_pub_.publish(odom);

    ROS_DEBUG("Published odometry: pos(%.3f,%.3f,%.3f) vel(%.3f,%.3f,%.3f)",
             x_, y_, th_, vx_, vy_, vth_);
}

void ChassisController::updateOdometry(double vx, double vy, double vth, double dt)
{
    // 批量更新速度
    vx_ = vx; vy_ = vy; vth_ = vth;

    // 缓存三角函数计算避免重复
    auto cos_th = cos(th_);
    auto sin_th = sin(th_);

    // 计算位置增量
    auto delta_x = (vx * cos_th - vy * sin_th) * dt;
    auto delta_y = (vx * sin_th + vy * cos_th) * dt;
    auto delta_th = vth * dt;

    // 批量更新位置
    x_ += delta_x; y_ += delta_y; th_ += delta_th;

    // 使用fmod优化角度归一化
    th_ = fmod(th_ + M_PI, 2.0 * M_PI) - M_PI;
}

bool ChassisController::initializeSerial()
{
    try {
        // 配置串口
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baudrate_);
        serial_port_.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
        serial_port_.setParity(serial::parity_none);
        serial_port_.setStopbits(serial::stopbits_one);
        serial_port_.setBytesize(serial::eightbits);

        // 打开串口
        serial_port_.open();

        if (!serial_port_.isOpen()) {
            ROS_ERROR("Failed to open serial port: %s", port_name_.c_str());
            serial_connected_ = false;
            return false;
        }

        ROS_INFO("Serial port opened successfully: %s", port_name_.c_str());
        serial_connected_ = true;
        reconnect_attempts_ = 0;
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize serial port: %s", e.what());
        serial_connected_ = false;
        return false;
    }
}

bool ChassisController::reconnectSerial()
{
    auto current_attempts = reconnect_attempts_.load();
    if (max_reconnect_attempts_ > 0 && current_attempts >= max_reconnect_attempts_) {
        ROS_ERROR("Max reconnect attempts (%d) reached", max_reconnect_attempts_);
        return false;
    }

    reconnect_attempts_++;
    ROS_WARN("Reconnecting... (attempt %d)", current_attempts + 1);

    try {
        if (serial_port_.isOpen()) serial_port_.close();
    } catch (const std::exception& e) {
        ROS_WARN("Close error: %s", e.what());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return initializeSerial();
}

bool ChassisController::isSerialConnected()
{
    try {
        return serial_port_.isOpen() && serial_connected_.load();
    } catch (const std::exception&) {
        serial_connected_ = false;
        return false;
    }
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
