#include "state.h"

State::State()
{

}

State::~State()
{

}

void State::setStateTime(std_msgs::Header& header)
{
    state_time_ = header.stamp.toSec();
}

void State::ConvertToPoseMsg(geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
    ros::Time time(state_time_);
    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = "camera_init";

    pose_msg.pose.pose.position.x = position_[0];
    pose_msg.pose.pose.position.y = position_[1];
    pose_msg.pose.pose.position.z = position_[2];

    pose_msg.pose.pose.orientation.w = attitude_.w();
    pose_msg.pose.pose.orientation.x = attitude_.x();
    pose_msg.pose.pose.orientation.y = attitude_.y();
    pose_msg.pose.pose.orientation.z = attitude_.z();

}