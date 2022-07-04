#ifndef _BOREALIS_FUSION_NODE_PARAM_H_
#define _BOREALIS_FUSION_NODE_PARAM_H_

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

class NodeParams
{
public:
    int n_states_;
    int n_control_inputs_;
    int n_measurement_states_;
  
    Eigen::MatrixXd process_transition_F_;
    Eigen::MatrixXd control_transition_B_;
    Eigen::MatrixXd observation_transition_H_;

    Eigen::MatrixXd initial_states_cov_P_;
    Eigen::MatrixXd sensor_noise_cov_R_;
    Eigen::MatrixXd process_noise_cov_Q_;

    NodeParams();
    ~NodeParams();

    void SetNodeParams(const ros::NodeHandle& node);
    
};

#endif // _BOREALIS_FUSION_NODE_PARAM_H_