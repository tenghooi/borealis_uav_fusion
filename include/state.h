#ifndef _BOREALIS_FUSION_STATE_H_
#define _BOREALIS_FUSION_STATE_H_

#include <iostream>
#include <vector>
#include <string>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Dense>

#define N_STATES 10 // number of states

class State
{
public:

    double state_time_;

    // The states for fusion
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Quaterniond attitude_;

    // IMU measurements as system inputs
    Eigen::Vector3d linear_accel_imu_;
    Eigen::Vector4d angular_vel_imu_;

    // Error states covariance
    Eigen::Matrix<double, N_STATES, N_STATES> state_cov_;

    State();
    ~State();

    void setStateTime(std_msgs::Header& header);
    void ConvertToPoseMsg(geometry_msgs::PoseWithCovarianceStamped& pose_msg); // geometry_msgs::PoseWithCovarianceStamped
    
};

#endif // _BOREALIS_FUSION_STATE_H_