#ifndef _BOREALIS_KALMAN_FILTER_H_
#define _BOREALIS_KALMAN_FILTER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>

#include <ros/ros.h>

#include <Eigen/Dense>

class KalmanFilter
{
protected:
    Eigen::MatrixXd state_transition_F_;
    Eigen::MatrixXd control_transition_B_;
    Eigen::MatrixXd observation_transition_H_;

    Eigen::Matrix<double, 10, 1> states_x_;
    Eigen::Matrix<double, 6, 1> control_u_;
    Eigen::VectorXd measurement_z_;

    Eigen::MatrixXd states_cov_P_;
    Eigen::MatrixXd process_noise_cov_Q_;
    Eigen::MatrixXd measurement_noise_cov_R_;

    Eigen::MatrixXd identity_I_;
    Eigen::MatrixXd kalman_gain_K_;

public:
    KalmanFilter();
    ~KalmanFilter();
    
    void set_states(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Quaterniond& attitude);
    void set_control_input(const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel);
    void set_measurement();
    void set_cov_Q();
    void set_cov_R();

    void get_states(Eigen::VectorXd& states) const;
    Eigen::MatrixXd get_B() const;

    void reset();
    void PropagatePositionAndVelocity();
    void PropagateQuaternion(const double& angular_vel_mag);

    void PropagateState();
    void Predict();
    void Update();

};

#endif // _BOREALIS_KALMAN_FILTER_H_