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
    Eigen::MatrixXd observation_H_;

    Eigen::VectorXd states_x_;
    Eigen::VectorXd control_u_;
    Eigen::VectorXd measurement_z_;

    Eigen::MatrixXd states_cov_P_;
    Eigen::MatrixXd process_noise_cov_Q_;
    Eigen::MatrixXd measurement_noise_cov_R_;


public:
    KalmanFilter();
    ~KalmanFilter();
    
    void set_states();
    void set_matrices();

    void get_states() const;
    void get_matrices();

    void Initialize();
    void Predict();
    void Update();

};

#endif // _BOREALIS_KALMAN_FILTER_H_