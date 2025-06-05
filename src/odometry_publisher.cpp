#include "dr100_chassis_driver/odometry_publisher.h"
#include <cmath>
#include <algorithm>
#include <cstring>

namespace dr100_chassis_driver {

OdometryPublisher::OdometryPublisher()
    : nh_(nullptr)
    , is_initialized_(false)
    , is_running_(false)
    , publish_enabled_(false)
    , x_(0.0)
    , y_(0.0)
    , th_(0.0)
    , vx_(0.0)
    , vy_(0.0)
    , vth_(0.0)
    , has_new_feedback_(false)
{
}

OdometryPublisher::~OdometryPublisher()
{
    stop();
}

bool OdometryPublisher::initialize(ros::NodeHandle& nh, const std::string& odom_topic,
                                 const std::string& odom_frame_id, const std::string& base_frame_id,
                                 double publish_rate)
{
    if (is_initialized_) {
        ROS_WARN("OdometryPublisher already initialized");
        return true;
    }

    nh_ = &nh;
    odom_topic_ = odom_topic;
    odom_frame_id_ = odom_frame_id;
    base_frame_id_ = base_frame_id;
    publish_rate_ = publish_rate;

    // 检查发布频率设置
    publish_enabled_ = publish_rate_ > 0.0;
    if (publish_enabled_) {
        publish_rate_ = std::max(0.1, std::min(1000.0, publish_rate_));
    }

    // 初始化ROS组件
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_, 1);

    if (publish_enabled_) {
        odom_timer_ = nh_->createTimer(ros::Duration(1.0 / publish_rate_),
                                     &OdometryPublisher::odomTimerCallback, this);
    }

    // 初始化时间戳
    last_odom_time_ = ros::Time::now();

    is_initialized_ = true;
    ROS_INFO("OdometryPublisher initialized: topic=%s, rate=%.1fHz", 
             odom_topic_.c_str(), publish_enabled_ ? publish_rate_ : 0.0);
    return true;
}

void OdometryPublisher::start()
{
    if (!is_initialized_) {
        ROS_WARN("OdometryPublisher not initialized");
        return;
    }

    is_running_ = true;
    ROS_INFO("OdometryPublisher started");
}

void OdometryPublisher::stop()
{
    if (!is_running_) return;

    is_running_ = false;
    ROS_INFO("OdometryPublisher stopped");
}

void OdometryPublisher::processFeedbackPacket(const FeedbackPacket& packet)
{
    if (!is_initialized_ || !is_running_) return;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_feedback_ = packet;
    has_new_feedback_ = true;

    // 如果不使用定时器发布，则立即发布
    if (!publish_enabled_) {
        publishOdometry();
    }
}

void OdometryPublisher::odomTimerCallback(const ros::TimerEvent& event)
{
    if (!is_initialized_ || !is_running_ || !publish_enabled_) return;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (has_new_feedback_) {
        publishOdometry();
        has_new_feedback_ = false;
    }
}

void OdometryPublisher::updateOdometry(double vx, double vy, double vth, double dt)
{
    // 批量更新速度
    vx_ = vx;
    vy_ = vy;
    vth_ = vth;

    // 缓存三角函数计算避免重复
    const auto cos_th = std::cos(th_);
    const auto sin_th = std::sin(th_);

    // 计算位置增量
    const auto delta_x = (vx * cos_th - vy * sin_th) * dt;
    const auto delta_y = (vx * sin_th + vy * cos_th) * dt;
    const auto delta_th = vth * dt;

    // 批量更新位置
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // 优化角度归一化（避免不必要的计算）
    if (th_ > M_PI) {
        th_ -= 2.0 * M_PI;
    } else if (th_ < -M_PI) {
        th_ += 2.0 * M_PI;
    }
}

void OdometryPublisher::publishOdometry()
{
    // 缓存时间计算
    const auto current_time = ros::Time::now();
    const auto dt = (current_time - last_odom_time_).toSec();
    last_odom_time_ = current_time;

    // 转换速度数据（使用常量避免除法）
    constexpr double inv_feedback_scale = 1.0 / FEEDBACK_VEL_SCALE;
    const auto vx = static_cast<double>(latest_feedback_.x_velocity) * inv_feedback_scale;
    const auto vy = static_cast<double>(latest_feedback_.y_velocity) * inv_feedback_scale;
    const auto vth = static_cast<double>(latest_feedback_.z_velocity) * inv_feedback_scale;

    updateOdometry(vx, vy, vth, dt);

    // 创建四元数（只计算一次）
    const auto odom_quat = tf::createQuaternionMsgFromYaw(th_);

    // 创建通用头部
    auto header = createHeader(odom_frame_id_);

    // 发布TF变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = header;
    odom_trans.child_frame_id = base_frame_id_;
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry odom;
    odom.header = std::move(header);
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

    // 优化协方差矩阵设置
    const bool is_stationary = (vx_ == 0.0 && vth_ == 0.0);
    const auto* pose_cov = is_stationary ? odom_pose_covariance2 : odom_pose_covariance;
    const auto* twist_cov = is_stationary ? odom_twist_covariance2 : odom_twist_covariance;

    std::memcpy(odom.pose.covariance.data(), pose_cov, 36 * sizeof(double));
    std::memcpy(odom.twist.covariance.data(), twist_cov, 36 * sizeof(double));

    odom_pub_.publish(odom);
}

std_msgs::Header OdometryPublisher::createHeader(const std::string& frame_id) const
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    return header;
}

} // namespace dr100_chassis_driver
