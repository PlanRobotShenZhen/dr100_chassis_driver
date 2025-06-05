#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <functional>
#include "dr100_chassis_driver/common_types.h"

namespace dr100_chassis_driver {

class SerialCommunication
{
public:
    // 回调函数类型定义
    using FeedbackCallback = std::function<void(const FeedbackPacket&)>;
    using ErrorCallback = std::function<void(const char*)>;

    SerialCommunication();
    ~SerialCommunication();

    // 初始化和控制
    bool initialize(const std::string& port_name, int baudrate, 
                   double reconnect_interval = 2.0, int max_reconnect_attempts = -1);
    void start();
    void stop();
    bool isConnected() const;

    // 数据发送
    bool sendControlPacket(const ControlPacket& packet);

    // 回调函数设置
    void setFeedbackCallback(const FeedbackCallback& callback);
    void setErrorCallback(const ErrorCallback& callback);

private:
    // 串口操作
    bool initializeSerial();
    bool reconnectSerial();
    void closeSerial();

    // 数据处理
    bool receiveFeedbackPacket(FeedbackPacket& packet);
    void processSerialData();
    uint8_t calculateChecksum(const uint8_t* data, size_t length);

    // 线程函数
    void serialProcessingThread();
    void reconnectionThread();

    // 错误处理
    void handleSerialError(const char* error_msg);

    // 串口相关
    serial::Serial serial_port_;
    std::string port_name_;
    int baudrate_;

    // 重连参数
    double reconnect_interval_;
    int max_reconnect_attempts_;
    std::atomic<int> reconnect_attempts_;
    ros::Time last_reconnect_attempt_;

    // 状态管理
    std::atomic<bool> is_initialized_;
    std::atomic<bool> is_running_;
    std::atomic<bool> shutdown_requested_;
    std::atomic<bool> serial_connected_;

    // 线程管理
    std::thread serial_thread_;
    std::thread reconnect_thread_;

    // 线程同步
    mutable std::mutex serial_mutex_;
    std::condition_variable reconnect_cv_;

    // 回调函数
    FeedbackCallback feedback_callback_;
    ErrorCallback error_callback_;
};

} // namespace dr100_chassis_driver

#endif // SERIAL_COMMUNICATION_H
