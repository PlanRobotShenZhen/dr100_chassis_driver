#include "dr100_chassis_driver/chassis_controller.h"
#include <cstring>
#include <algorithm>
#include <cmath>

ChassisController::ChassisController()
    : private_nh_("~")
    , is_initialized_(false)
    , motor_enable_(true)
    , reconnect_attempts_(0)
    , serial_connected_(false)
    , x_(0.0)
    , y_(0.0)
    , th_(0.0)
    , vx_(0.0)
    , vy_(0.0)
    , vth_(0.0)
{
    // 获取参数
    private_nh_.param<std::string>("port", port_name_, "/tmp/ttyV1");
    private_nh_.param<int>("baudrate", baudrate_, 115200);
    private_nh_.param<double>("max_linear_velocity", max_linear_velocity_, 2.0);
    private_nh_.param<double>("max_angular_velocity", max_angular_velocity_, 2.0);
    private_nh_.param<double>("cmd_timeout", cmd_timeout_, 1.0);
    private_nh_.param<bool>("motor_enable", motor_enable_, true);
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    private_nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    private_nh_.param<double>("reconnect_interval", reconnect_interval_, 2.0);
    private_nh_.param<int>("max_reconnect_attempts", max_reconnect_attempts_, -1); // -1表示无限重试
    private_nh_.param<double>("odom_publish_rate", odom_publish_rate_, 50.0); // 默认50Hz

    // 检查里程计发布频率范围并设置发布标志
    if (odom_publish_rate_ <= 0.0) {
        publish_odom_ = false;
        ROS_WARN("Odometry publish rate <= 0, odometry publishing disabled");
    } else {
        publish_odom_ = true;
        // 限制频率范围：0.1Hz到1000Hz
        if (odom_publish_rate_ < 0.1) {
            odom_publish_rate_ = 0.1;
            ROS_WARN("Odometry publish rate too low, set to 0.1 Hz");
        } else if (odom_publish_rate_ > 1000.0) {
            odom_publish_rate_ = 1000.0;
            ROS_WARN("Odometry publish rate too high, set to 1000 Hz");
        }
    }
    
    ROS_INFO("ChassisController initialized with port: %s, baudrate: %d",
             port_name_.c_str(), baudrate_);
    ROS_INFO("Topics: cmd_vel=%s, odom=%s", cmd_vel_topic_.c_str(), odom_topic_.c_str());
    ROS_INFO("Frames: odom_frame=%s, base_frame=%s", odom_frame_id_.c_str(), base_frame_id_.c_str());
    if (publish_odom_) {
        ROS_INFO("Odometry publish rate: %.1f Hz", odom_publish_rate_);
    } else {
        ROS_INFO("Odometry publishing disabled");
    }
}

ChassisController::~ChassisController()
{
    if (serial_port_.isOpen()) {
        serial_port_.close();
    }
}

bool ChassisController::initialize()
{
    try {
        // 初始化串口
        if (!initializeSerial()) {
            ROS_ERROR("Failed to initialize serial port");
            return false;
        }

        // 初始化ROS话题
        cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &ChassisController::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 1);

        is_initialized_ = true;
        last_cmd_time_ = ros::Time::now();
        last_odom_time_ = ros::Time::now();
        last_odom_publish_time_ = ros::Time::now();
        last_reconnect_attempt_ = ros::Time::now();

        ROS_INFO("ChassisController initialized successfully");
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize ChassisController: %s", e.what());
        return false;
    }
}

void ChassisController::run()
{
    if (!is_initialized_) {
        ROS_ERROR("ChassisController not initialized");
        return;
    }

    ros::Rate rate(50); // 50Hz

    while (ros::ok()) {
        // 检查串口连接状态
        if (!isSerialConnected()) {
            // 尝试重连
            if ((ros::Time::now() - last_reconnect_attempt_).toSec() >= reconnect_interval_) {
                if (reconnectSerial()) {
                    ROS_INFO("Serial port reconnected successfully");
                    reconnect_attempts_ = 0;
                } else {
                    last_reconnect_attempt_ = ros::Time::now();
                }
            }
        } else {
            // 处理串口数据
            processSerialData();

            // 检查命令超时
            if ((ros::Time::now() - last_cmd_time_).toSec() > cmd_timeout_) {
                // 发送停止命令
                ControlPacket stop_packet = createControlPacket(0.0, 0.0, 0.0);
                sendControlPacket(stop_packet);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (!is_initialized_) {
        return;
    }
    
    // 限制速度范围
    double linear_x = std::max(-max_linear_velocity_, 
                              std::min(max_linear_velocity_, msg->linear.x));
    double linear_y = std::max(-max_linear_velocity_, 
                              std::min(max_linear_velocity_, msg->linear.y));
    double angular_z = std::max(-max_angular_velocity_, 
                               std::min(max_angular_velocity_, msg->angular.z));
    
    // 创建控制数据包
    ControlPacket packet = createControlPacket(linear_x, linear_y, angular_z);
    
    // 发送数据包
    if (sendControlPacket(packet)) {
        last_cmd_time_ = ros::Time::now();
        ROS_DEBUG("Sent velocity command: linear_x=%.3f, linear_y=%.3f, angular_z=%.3f", 
                 linear_x, linear_y, angular_z);
    } else {
        ROS_WARN("Failed to send velocity command");
    }
}

ChassisController::ControlPacket ChassisController::createControlPacket(
    double linear_x, double linear_y, double angular_z)
{
    ControlPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    packet.frame_header = 0x7B;
    packet.motor_enable = motor_enable_ ? 1 : 0;
    packet.emergency_stop = 0;  // 0表示松开
    packet.light_control = 0;   // 0表示灭
    
    // 速度转换：放大1000倍
    packet.x_velocity = static_cast<int16_t>(linear_x * 1000);
    packet.y_velocity = static_cast<int16_t>(linear_y * 1000);
    packet.z_velocity = static_cast<int16_t>(angular_z * 1000);
    
    packet.ultrasonic_switch = 0;
    packet.charge_switch = 0;
    packet.lidar_switch = 0;
    packet.reserved1 = 0;
    packet.reserved2 = 0;
    
    // 计算校验码（对前15个字节进行异或运算）
    packet.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&packet), 15);
    packet.frame_tail = 0x7D;
    
    return packet;
}

uint8_t ChassisController::calculateChecksum(const uint8_t* data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

bool ChassisController::sendControlPacket(const ControlPacket& packet)
{
    try {
        if (!isSerialConnected()) {
            ROS_DEBUG("Serial port is not connected, cannot send packet");
            return false;
        }

        size_t bytes_written = serial_port_.write(reinterpret_cast<const uint8_t*>(&packet),
                                                 sizeof(packet));

        if (bytes_written != sizeof(packet)) {
            ROS_WARN("Incomplete packet sent: %zu/%zu bytes", bytes_written, sizeof(packet));
            // 标记串口为断开状态
            serial_connected_ = false;
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to send control packet: %s", e.what());
        // 标记串口为断开状态
        serial_connected_ = false;
        return false;
    }
}

void ChassisController::processSerialData()
{
    try {
        if (!isSerialConnected()) {
            return;
        }

        // 检查是否有足够的数据
        size_t available = serial_port_.available();
        if (available < sizeof(FeedbackPacket)) {
            return;
        }

        FeedbackPacket packet;
        if (receiveFeedbackPacket(packet)) {
            publishOdometry(packet);
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Error processing serial data: %s", e.what());
        // 标记串口为断开状态
        serial_connected_ = false;
    }
}

bool ChassisController::receiveFeedbackPacket(FeedbackPacket& packet)
{
    try {
        // 寻找帧头
        uint8_t byte;
        bool found_header = false;

        while (serial_port_.available() > 0 && !found_header) {
            serial_port_.read(&byte, 1);
            if (byte == 0x7B) {
                found_header = true;
                packet.frame_header = byte;
            }
        }

        if (!found_header) {
            return false;
        }

        // 读取剩余数据
        size_t remaining_bytes = sizeof(FeedbackPacket) - 1;
        if (serial_port_.available() < remaining_bytes) {
            return false;
        }

        uint8_t* packet_ptr = reinterpret_cast<uint8_t*>(&packet) + 1;
        size_t bytes_read = serial_port_.read(packet_ptr, remaining_bytes);

        if (bytes_read != remaining_bytes) {
            ROS_WARN("Incomplete feedback packet received: %zu/%zu bytes",
                     bytes_read, remaining_bytes);
            return false;
        }

        // 验证帧尾
        if (packet.frame_tail != 0x7D) {
            ROS_WARN("Invalid frame tail: 0x%02X", packet.frame_tail);
            return false;
        }

        // 验证校验码
        uint8_t calculated_checksum = calculateChecksum(
            reinterpret_cast<const uint8_t*>(&packet), sizeof(FeedbackPacket) - 2);

        if (packet.checksum != calculated_checksum) {
            ROS_WARN("Checksum mismatch: received=0x%02X, calculated=0x%02X",
                     packet.checksum, calculated_checksum);
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to receive feedback packet: %s", e.what());
        // 标记串口为断开状态
        serial_connected_ = false;
        return false;
    }
}

void ChassisController::publishOdometry(const FeedbackPacket& packet)
{
    ros::Time current_time = ros::Time::now();

    // 从反馈包中提取速度信息（缩小100倍）
    double vx = static_cast<double>(packet.x_velocity) / 100.0;
    double vy = static_cast<double>(packet.y_velocity) / 100.0;
    double vth = static_cast<double>(packet.z_velocity) / 100.0;

    // 计算时间差
    double dt = (current_time - last_odom_time_).toSec();
    last_odom_time_ = current_time;

    // 更新里程计
    updateOdometry(vx, vy, vth, dt);

    // 检查是否需要发布里程计
    if (!publish_odom_) {
        return; // 不发布里程计
    }

    // 检查发布频率
    double time_since_last_publish = (current_time - last_odom_publish_time_).toSec();
    double publish_interval = 1.0 / odom_publish_rate_;

    if (time_since_last_publish < publish_interval) {
        return; // 还没到发布时间
    }

    last_odom_publish_time_ = current_time;

    // 创建四元数
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

    // 发布TF变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id_;
    odom_trans.child_frame_id = base_frame_id_;

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;

    // 位置
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // 速度
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;

    // 设置协方差矩阵
    if (vx_ == 0 && vth_ == 0) {
        // 静止时使用更高精度的协方差矩阵
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = odom_pose_covariance2[i];
            odom.twist.covariance[i] = odom_twist_covariance2[i];
        }
    } else {
        // 运动时使用标准协方差矩阵
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = odom_pose_covariance[i];
            odom.twist.covariance[i] = odom_twist_covariance[i];
        }
    }

    odom_pub_.publish(odom);

    ROS_DEBUG("Published odometry: pos(%.3f,%.3f,%.3f) vel(%.3f,%.3f,%.3f)",
             x_, y_, th_, vx_, vy_, vth_);
}

void ChassisController::updateOdometry(double vx, double vy, double vth, double dt)
{
    // 更新速度
    vx_ = vx;
    vy_ = vy;
    vth_ = vth;

    // 计算位置增量
    double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
    double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
    double delta_th = vth * dt;

    // 更新位置
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // 角度归一化到[-π, π]
    while (th_ > M_PI) th_ -= 2.0 * M_PI;
    while (th_ < -M_PI) th_ += 2.0 * M_PI;
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
    // 检查是否达到最大重试次数
    if (max_reconnect_attempts_ > 0 && reconnect_attempts_ >= max_reconnect_attempts_) {
        ROS_ERROR("Maximum reconnection attempts (%d) reached, giving up", max_reconnect_attempts_);
        return false;
    }

    reconnect_attempts_++;
    ROS_WARN("Attempting to reconnect serial port (attempt %d/%d)...",
             reconnect_attempts_, max_reconnect_attempts_ > 0 ? max_reconnect_attempts_ : -1);

    // 关闭现有连接
    try {
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
    } catch (const std::exception& e) {
        ROS_WARN("Error closing serial port: %s", e.what());
    }

    // 短暂延迟
    ros::Duration(0.5).sleep();

    // 尝试重新初始化
    return initializeSerial();
}

bool ChassisController::isSerialConnected()
{
    try {
        // 检查串口是否打开且连接状态正常
        if (!serial_port_.isOpen() || !serial_connected_) {
            return false;
        }

        // 可以添加额外的连接检查逻辑
        return true;

    } catch (const std::exception& e) {
        ROS_DEBUG("Serial connection check failed: %s", e.what());
        serial_connected_ = false;
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_controller");

    ChassisController controller;

    if (!controller.initialize()) {
        ROS_ERROR("Failed to initialize chassis controller");
        return -1;
    }

    ROS_INFO("Chassis controller started");
    controller.run();

    return 0;
}
