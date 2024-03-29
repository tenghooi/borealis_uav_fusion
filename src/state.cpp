#include "state.h"

State::State()
{
    
}

State::~State()
{

}

void State::reset()
{
    state_time_ = 0.0;

    position_.setZero();
    velocity_.setZero();
    attitude_.setIdentity();

    linear_accel_imu_.setZero();
    angular_vel_imu_.setZero();

    state_cov_.setIdentity();
}

void State::setStateTime(const std_msgs::Header& header)
{
    state_time_ = header.stamp.toSec();
}

void State::ConvertToPoseMsg(geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
    ros::Time time(state_time_);
    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = "os_sensor";

    // pose_msg.pose.pose.position.x = position_[0];
    // pose_msg.pose.pose.position.y = position_[1];
    // pose_msg.pose.pose.position.z = position_[2];
    pose_msg.pose.pose.position.x = 0;
    pose_msg.pose.pose.position.y = 0;
    pose_msg.pose.pose.position.z = 0;

    pose_msg.pose.pose.orientation.w = attitude_.w();
    pose_msg.pose.pose.orientation.x = attitude_.x();
    pose_msg.pose.pose.orientation.y = attitude_.y();
    pose_msg.pose.pose.orientation.z = attitude_.z();

}