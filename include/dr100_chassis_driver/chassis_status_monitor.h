#ifndef CHASSIS_STATUS_MONITOR_H
#define CHASSIS_STATUS_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <mutex>
#include <atomic>
#include "dr100_chassis_driver/common_types.h"

namespace dr100_chassis_driver {

class ChassisStatusMonitor
{
public:
    ChassisStatusMonitor();
    ~ChassisStatusMonitor();

    // 初始化和控制
    bool initialize(ros::NodeHandle& nh, 
                   const std::string& motor_enable_status_topic = "/chassis/motor_enable_status",
                   const std::string& fault_status_topic = "/chassis/fault_status", 
                   const std::string& robot_status_topic = "/chassis/robot_status",
                   const std::string& diagnostics_topic = "/chassis/diagnostics",
                   double publish_rate = 10.0);
    void start();
    void stop();

    // 数据处理
    void processFeedbackPacket(const FeedbackPacket& packet);

    // 状态查询
    bool isInitialized() const { return is_initialized_; }
    bool isRunning() const { return is_running_; }
    
    // 获取当前状态
    MotorEnableStatus getMotorEnableStatus() const { return current_motor_enable_status_; }
    FaultStatus getFaultStatus() const { return current_fault_status_; }
    RobotSystemStatus getRobotSystemStatus() const { return current_robot_status_; }

private:
    // ROS相关
    ros::NodeHandle* nh_;
    ros::Publisher motor_enable_status_pub_;
    ros::Publisher fault_status_pub_;
    ros::Publisher robot_status_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Timer status_timer_;

    // 配置参数
    std::string motor_enable_status_topic_;
    std::string fault_status_topic_;
    std::string robot_status_topic_;
    std::string diagnostics_topic_;
    double publish_rate_;
    bool publish_enabled_;

    // 状态管理
    std::atomic<bool> is_initialized_;
    std::atomic<bool> is_running_;

    // 数据同步
    std::mutex status_mutex_;
    FeedbackPacket latest_feedback_;
    bool has_new_feedback_;

    // 当前状态
    std::atomic<MotorEnableStatus> current_motor_enable_status_;
    std::atomic<FaultStatus> current_fault_status_;
    std::atomic<RobotSystemStatus> current_robot_status_;

    // 状态变化检测
    MotorEnableStatus last_motor_enable_status_;
    FaultStatus last_fault_status_;
    RobotSystemStatus last_robot_status_;

    // 内部方法
    void statusTimerCallback(const ros::TimerEvent& event);
    void publishChassisStatus();
    void updateStatusFromPacket(const FeedbackPacket& packet);
    void publishDiagnostics();
    
    // 状态转换辅助函数
    MotorEnableStatus convertMotorEnableStatus(uint8_t raw_status) const;
    FaultStatus convertFaultStatus(uint8_t raw_status) const;
    RobotSystemStatus convertRobotSystemStatus(uint8_t raw_status) const;
    
    // 诊断信息辅助函数
    std::string getMotorEnableStatusString(MotorEnableStatus status) const;
    std::string getFaultStatusString(FaultStatus status) const;
    std::string getRobotSystemStatusString(RobotSystemStatus status) const;
    uint8_t getDiagnosticLevel(FaultStatus fault_status, RobotSystemStatus robot_status) const;
};

} // namespace dr100_chassis_driver

#endif // CHASSIS_STATUS_MONITOR_H
