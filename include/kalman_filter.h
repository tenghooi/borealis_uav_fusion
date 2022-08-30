#ifndef _BOREALIS_KALMAN_FILTER_H_
#define _BOREALIS_KALMAN_FILTER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>

#include <ros/ros.h>

#include <Eigen/Dense>

class KalmanFilter
{
protected:
    Eigen::MatrixXd state_transition_F_;
    Eigen::MatrixXd control_transition_B_;
    Eigen::MatrixXd observation_transition_H_;

    Eigen::VectorXd states_x_;
    Eigen::VectorXd control_u_;
    Eigen::VectorXd measurement_z_;

    Eigen::MatrixXd states_cov_P_;
    Eigen::MatrixXd process_noise_cov_Q_;
    Eigen::MatrixXd measurement_noise_cov_R_;

    Eigen::MatrixXd identity_I_;
    Eigen::MatrixXd kalman_gain_K_;

public:
    KalmanFilter();
    ~KalmanFilter();
    
    void set_states(Eigen::VectorXd position, Eigen::VectorXd velocity, Eigen::Quaterniond attitude);
    void set_control_input(Eigen::VectorXd linear_accel, Eigen::VectorXd angular_vel);
    void set_measurement();
    void set_cov_Q();
    void set_cov_R();

    void get_states() const;
    void get_matrices();

    void Initialize();
    void PropagateState();
    void Predict();
    void Update();

};

#endif // _BOREALIS_KALMAN_FILTER_H_