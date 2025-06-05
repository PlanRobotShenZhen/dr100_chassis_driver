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

    if (!initializeSerial()) {
        ROS_ERROR("Failed to initialize serial port: %s", port_name_.c_str());
        return false;
    }

    is_initialized_ = true;
    ROS_INFO("SerialCommunication initialized: port=%s, baudrate=%d", 
             port_name_.c_str(), baudrate_);
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

        constexpr size_t packet_size = CONTROL_PACKET_SIZE;
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

bool SerialCommunication::reconnectSerial()
{
    auto current_attempts = reconnect_attempts_.load();
    if (max_reconnect_attempts_ > 0 && current_attempts >= max_reconnect_attempts_) {
        ROS_ERROR("Max reconnect attempts (%d) reached", max_reconnect_attempts_);
        return false;
    }

    reconnect_attempts_++;
    ROS_WARN("Reconnecting... (attempt %d)", current_attempts + 1);

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
        constexpr size_t MIN_AVAILABLE = FEEDBACK_PACKET_SIZE;

        // 快速检查可用数据
        if (serial_port_.available() < MIN_AVAILABLE) return false;

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

        // 快速验证
        if (bytes_read != REMAINING_BYTES || packet.frame_tail != FRAME_TAIL) {
            return false;
        }

        // 校验码验证
        uint8_t calculated_checksum = calculateChecksum(
            reinterpret_cast<const uint8_t*>(&packet), FEEDBACK_CHECKSUM_LENGTH);

        return packet.checksum == calculated_checksum;

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
        if (receiveFeedbackPacket(packet) && feedback_callback_) {
            feedback_callback_(packet);
        }

    } catch (const std::exception&) {
        handleSerialError("Process serial failed");
    }
}

void SerialCommunication::serialProcessingThread()
{
    ROS_INFO("Serial processing thread started");
    ros::Rate rate(100);

    while (ros::ok() && !shutdown_requested_) {
        if (isConnected()) {
            processSerialData();
        }
        rate.sleep();
    }

    ROS_INFO("Serial processing thread exiting");
}

void SerialCommunication::reconnectionThread()
{
    ROS_INFO("Reconnection thread started");

    while (ros::ok() && !shutdown_requested_) {
        std::unique_lock<std::mutex> lock(serial_mutex_);

        auto timeout = std::chrono::seconds(static_cast<int>(reconnect_interval_));
        if (reconnect_cv_.wait_for(lock, timeout, [this] {
            return shutdown_requested_ || !isConnected();
        })) {
            if (shutdown_requested_) break;

            if (!isConnected() && reconnectSerial()) {
                ROS_INFO("Serial reconnected");
                reconnect_attempts_ = 0;
            }
        }
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

} // namespace dr100_chassis_driver
