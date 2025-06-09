#ifndef ODOMETRY_PUBLISHER_H
#define ODOMETRY_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <mutex>
#include <atomic>
#include "dr100_chassis_driver/common_types.h"

namespace dr100_chassis_driver {

class OdometryPublisher
{
public:
    OdometryPublisher();
    ~OdometryPublisher();

    // 初始化和控制
    bool initialize(ros::NodeHandle& nh, const std::string& odom_topic,
                   const std::string& odom_frame_id, const std::string& base_frame_id,
                   double publish_rate = 50.0, bool publish_tf = true);
    void start();
    void stop();

    // 数据处理
    void processFeedbackPacket(const FeedbackPacket& packet);

private:
    // 里程计计算
    void updateOdometry(double vx, double vy, double vth, double dt);
    void publishOdometry();

    // 定时器回调
    void odomTimerCallback(const ros::TimerEvent& event);

    // 辅助函数
    std_msgs::Header createHeader(const std::string& frame_id) const;

    // ROS相关
    ros::NodeHandle* nh_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;
    ros::Timer odom_timer_;

    // 参数
    std::string odom_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    double publish_rate_;
    bool publish_enabled_;
    bool publish_tf_;

    // 状态管理
    std::atomic<bool> is_initialized_;
    std::atomic<bool> is_running_;

    // 里程计状态 (需要线程保护)
    mutable std::mutex odom_mutex_;
    double x_;
    double y_;
    double th_;
    double vx_;
    double vy_;
    double vth_;
    ros::Time last_odom_time_;

    // 最新的反馈数据
    FeedbackPacket latest_feedback_;
    bool has_new_feedback_;
};

} // namespace dr100_chassis_driver

#endif // ODOMETRY_PUBLISHER_H
