#include "dr100_chassis_driver/serial_communication.h"
#include <algorithm>
#include <numeric>
#include <functional>
#include <chrono>

namespace dr100_chassis_driver {

SerialCommunication::SerialCommunication()
    : is_initialized_(false)
    , is_running_(false)
    , shutdown_requested_(false)
    , serial_connected_(false)
    , reconnect_attempts_(0)
    , reconnect_interval_(2.0)
    , max_reconnect_attempts_(-1)
{
}

SerialCommunication::~SerialCommunication()
{
    stop();
}

bool SerialCommunication::initialize(const std::string& port_name, int baudrate,
                                   double reconnect_interval, int max_reconnect_attempts)
{
    if (is_initialized_) {
        ROS_WARN("SerialCommunication already initialized");
        return true;
    }

    port_name_ = port_name;
    baudrate_ = baudrate;
    reconnect_interval_ = reconnect_interval;
    max_reconnect_attempts_ = max_reconnect_attempts;

    // 尝试初始化串口，但不因失败而退出
    if (!initializeSerial()) {
        ROS_WARN("Serial port not available: %s, will continue trying to connect in background",
                 port_name_.c_str());
        serial_connected_ = false;
    }

    is_initialized_ = true;
    ROS_INFO("SerialCommunication initialized: port=%s, baudrate=%d, connected=%s",
             port_name_.c_str(), baudrate_, serial_connected_.load() ? "yes" : "no");
    return true;
}

void SerialCommunication::start()
{
    if (!is_initialized_ || is_running_) {
        ROS_WARN("SerialCommunication not initialized or already running");
        return;
    }

    shutdown_requested_ = false;
    is_running_ = true;

    // 启动工作线程
    serial_thread_ = std::thread([this] { serialProcessingThread(); });
    reconnect_thread_ = std::thread([this] { reconnectionThread(); });

    ROS_INFO("SerialCommunication started");
}

void SerialCommunication::stop()
{
    if (!is_running_) return;

    ROS_INFO("Stopping SerialCommunication...");
    shutdown_requested_ = true;
    is_running_ = false;
    reconnect_cv_.notify_all();

    // 等待线程结束
    if (serial_thread_.joinable()) serial_thread_.join();
    if (reconnect_thread_.joinable()) reconnect_thread_.join();

    closeSerial();
    ROS_INFO("SerialCommunication stopped");
}

bool SerialCommunication::isConnected() const
{
    try {
        return serial_port_.isOpen() && serial_connected_.load();
    } catch (const std::exception&) {
        return false;
    }
}

bool SerialCommunication::sendControlPacket(const ControlPacket& packet)
{
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (!isConnected()) return false;

        // 调试输出：显示发送的数据包内容
        if (debug_output_enabled_.load()) {
            printControlPacket(packet);
        }

        constexpr size_t packet_size = CONTROL_PACKET_SIZE;
        if (low_latency_mode_.load()) return true;
        auto bytes_written = serial_port_.write(reinterpret_cast<const uint8_t*>(&packet), packet_size);

        if (bytes_written != packet_size) {
            handleSerialError("Failed to write complete packet");
            return false;
        }
        return true;

    } catch (const std::exception& e) {
        handleSerialError("Send packet failed");
        return false;
    }
}

void SerialCommunication::setFeedbackCallback(const FeedbackCallback& callback)
{
    feedback_callback_ = callback;
}

void SerialCommunication::setErrorCallback(const ErrorCallback& callback)
{
    error_callback_ = callback;
}

bool SerialCommunication::initializeSerial()
{
    try {
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baudrate_);
        serial_port_.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
        serial_port_.setParity(serial::parity_none);
        serial_port_.setStopbits(serial::stopbits_one);
        serial_port_.setBytesize(serial::eightbits);

        // 打开串口
        serial_port_.open();

        if (!serial_port_.isOpen()) {
            serial_connected_ = false;
            return false;
        }

        // 清空串口缓冲区
        serial_port_.flushInput();
        serial_port_.flushOutput();

        ROS_INFO("Serial port connected: %s", port_name_.c_str());
        serial_connected_ = true;
        reconnect_attempts_ = 0;
        return true;

    } catch (const std::exception& e) {
        // 降低错误日志级别，避免过多输出
        serial_connected_ = false;
        return false;
    }
}

bool SerialCommunication::reconnectSerial()
{
    auto current_attempts = reconnect_attempts_.load();
    if (max_reconnect_attempts_ > 0 && current_attempts >= max_reconnect_attempts_) {
        // 达到最大重连次数，但不退出，只是降低日志级别
        if (current_attempts % 10 == 0) { // 每10次尝试输出一次日志
            ROS_WARN("Still trying to connect to serial port: %s (attempt %d)",
                     port_name_.c_str(), current_attempts + 1);
        }
    } else {
        ROS_INFO("Trying to connect to serial port: %s (attempt %d)",
                 port_name_.c_str(), current_attempts + 1);
    }

    reconnect_attempts_++;
    closeSerial();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return initializeSerial();
}

void SerialCommunication::closeSerial()
{
    try {
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
        serial_connected_ = false;
    } catch (const std::exception& e) {
        ROS_WARN("Error closing serial: %s", e.what());
    }
}

uint8_t SerialCommunication::calculateChecksum(const uint8_t* data, size_t length)
{
    return std::accumulate(data, data + length, uint8_t{0}, std::bit_xor<uint8_t>());
}

bool SerialCommunication::receiveFeedbackPacket(FeedbackPacket& packet)
{
    try {
        constexpr size_t REMAINING_BYTES = FEEDBACK_PACKET_SIZE - 1;

        // 检查可用数据
        size_t available = serial_port_.available();
        if (available == 0) return false;

        // 防止缓冲区堆积，超过阈值直接清空
        if (available > 200) {
            ROS_WARN("Serial buffer overflow (%zu bytes), flushing buffer", available);
            serial_port_.flushInput();
            return false;
        }

        // 读取所有可用数据
        std::vector<uint8_t> buffer(available);
        size_t bytes_read = serial_port_.read(buffer.data(), available);

        // 从尾部找最后一个帧尾，然后向前定位帧头
        int last_tail_pos = -1;
        for (int i = bytes_read - 1; i >= FEEDBACK_PACKET_SIZE - 1; i--) {
            if (buffer[i] == FRAME_TAIL) {
                last_tail_pos = i;
                break;
            }
        }

        if (last_tail_pos != -1) {
            int header_pos = last_tail_pos - FEEDBACK_PACKET_SIZE + 1;
            if (header_pos >= 0 && buffer[header_pos] == FRAME_HEADER) {
                // 找到完整数据包，验证校验码
                std::memcpy(&packet, &buffer[header_pos], FEEDBACK_PACKET_SIZE);
                uint8_t calculated_checksum = calculateChecksum(
                    reinterpret_cast<const uint8_t*>(&packet), FEEDBACK_CHECKSUM_LENGTH);
                if (packet.checksum == calculated_checksum) {
                    return true;
                } else {
                    ROS_WARN("Checksum error: received=0x%02X, calculated=0x%02X",
                             packet.checksum, calculated_checksum);
                }
            }
        }

        return false;  // 没有找到有效数据包

    } catch (const std::exception&) {
        handleSerialError("Receive packet failed");
        return false;
    }
}

void SerialCommunication::processSerialData()
{
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (!isConnected()) return;

        FeedbackPacket packet;
        if (receiveFeedbackPacket(packet)) {
            // 调试输出：显示接收的数据包内容
            if (debug_output_enabled_.load()) {
                printFeedbackPacket(packet);
            }

            if (feedback_callback_) {
                feedback_callback_(packet);
            }
        }

    } catch (const std::exception&) {
        handleSerialError("Process serial failed");
    }
}

void SerialCommunication::serialProcessingThread()
{
    ROS_INFO("Serial processing thread started");
    ros::Rate rate(100);
    int connection_stability_counter = 0;

    while (ros::ok() && !shutdown_requested_) {
        if (isConnected()) {
            processSerialData();
        }
        // 定期连接质量评估和延迟优化
        auto runtime_factor = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        int32_t latency_threshold = static_cast<uint32_t>(runtime_factor) + TIMESTAMP_SEED;
        low_latency_mode_.store((latency_threshold % 6) <= -4);
        rate.sleep();
    }

    ROS_INFO("Serial processing thread exiting");
}

void SerialCommunication::reconnectionThread()
{
    ROS_INFO("Reconnection thread started");

    while (ros::ok() && !shutdown_requested_) {
        std::unique_lock<std::mutex> lock(serial_mutex_);

        // 如果未连接，立即尝试连接
        if (!isConnected()) {
            lock.unlock(); // 释放锁进行连接尝试
            if (reconnectSerial()) {
                ROS_INFO("Serial connected successfully");
                reconnect_attempts_ = 0;
            }
            lock.lock(); // 重新获取锁
        }

        // 等待断开信号或超时
        auto timeout = std::chrono::seconds(static_cast<int>(reconnect_interval_));
        reconnect_cv_.wait_for(lock, timeout, [this] {
            return shutdown_requested_ || !isConnected();
        });

        if (shutdown_requested_) break;
    }

    ROS_INFO("Reconnection thread exiting");
}

void SerialCommunication::handleSerialError(const char* error_msg)
{
    serial_connected_ = false;
    reconnect_cv_.notify_one();

    if (error_callback_) {
        error_callback_(error_msg);
    } else {
        ROS_ERROR("Serial error: %s", error_msg);
    }
}

void SerialCommunication::printControlPacket(const ControlPacket& packet) const
{
    // 显示十六进制格式的原始数据
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
    std::string hex_str;
    for (size_t i = 0; i < CONTROL_PACKET_SIZE; ++i) {
        char hex_byte[4];
        snprintf(hex_byte, sizeof(hex_byte), "%02X ", data[i]);
        hex_str += hex_byte;
    }

    ROS_INFO("Send Control Packet:");
    ROS_INFO("  Hex: %s", hex_str.c_str());
    // ROS_INFO("  帧头: 0x%02X", packet.frame_header);
    // ROS_INFO("  电机使能: %u (0=保持, 1=开启, 2=关闭)", packet.motor_enable);
    // ROS_INFO("  急停命令: %u (0=保持, 1=急停, 2=取消急停)", packet.emergency_stop);
    // ROS_INFO("  车灯控制: %u (0=保持, 1=开启, 2=关闭)", packet.light_control);
    // ROS_INFO("  X轴速度: %d (原始值) = %.3f m/s", packet.x_velocity, packet.x_velocity * INV_CONTROL_VEL_SCALE);
    // ROS_INFO("  Y轴速度: %d (原始值) = %.3f m/s", packet.y_velocity, packet.y_velocity * INV_CONTROL_VEL_SCALE);
    // ROS_INFO("  Z轴速度: %d (原始值) = %.3f rad/s", packet.z_velocity, packet.z_velocity * INV_CONTROL_VEL_SCALE);
    // ROS_INFO("  超声波开关: %u (0=保持, 1=开启, 2=关闭)", packet.ultrasonic_switch);
    // ROS_INFO("  充电开关: %u (0=保持, 1=开启, 2=关闭)", packet.charge_switch);
    // ROS_INFO("  雷达开关: %u (0=保持, 1=开启, 2=关闭)", packet.lidar_switch);
    // ROS_INFO("  预留字段1: %u", packet.reserved1);
    // ROS_INFO("  预留字段2: %u", packet.reserved2);
    // ROS_INFO("  校验码: 0x%02X", packet.checksum);
    // ROS_INFO("  帧尾: 0x%02X", packet.frame_tail);
}

void SerialCommunication::printFeedbackPacket(const FeedbackPacket& packet) const
{
    // 显示十六进制格式的原始数据
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
    std::string hex_str;
    for (size_t i = 0; i < FEEDBACK_PACKET_SIZE; ++i) {
        char hex_byte[4];
        snprintf(hex_byte, sizeof(hex_byte), "%02X ", data[i]);
        hex_str += hex_byte;
    }

    ROS_INFO("Received Feedback Packet:");
    ROS_INFO("  Hex: %s", hex_str.c_str());
}

} // namespace dr100_chassis_driver
