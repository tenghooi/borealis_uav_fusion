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
    Eigen::MatrixXd state_transition_F;
    Eigen::MatrixXd control_transition_B;
    Eigen::MatrixXd observation_H;

    Eigen::VectorXd states_x;
    Eigen::VectorXd control_u;
    Eigen::VectorXd measurement_z;

    Eigen::MatrixXd states_cov_P;
    Eigen::MatrixXd process_noise_cov_Q;
    Eigen::MatrixXd measurement_noise_cov_R;

    
public:


};

#endif // _BOREALIS_KALMAN_FILTER_H_