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
    std::string frame_id;

    int n_states_;
    int n_control_inputs_;
    int n_measurement_states_;
    
    Eigen::MatrixXd process_transition_F_;
    Eigen::MatrixXd control_transition_B_;
    Eigen::MatrixXd observation_transition_H_;
    // for parsing yaml and later to matrix
    std::vector<double> F_vector_form_; 
    std::vector<double> B_vector_form_;
    std::vector<double> H_vector_form_;

    Eigen::MatrixXd initial_states_cov_P_;
    Eigen::MatrixXd sensor_noise_cov_R_;
    Eigen::MatrixXd process_noise_cov_Q_;
    // for parsing yaml and later to matrix
    std::vector<double> P_vector_form_; 
    std::vector<double> R_vector_form_;
    std::vector<double> Q_vector_form_;

    NodeParams();
    ~NodeParams();

    void SetNodeParams(const ros::NodeHandle& node);
    
};

#endif // _BOREALIS_FUSION_NODE_PARAM_H_