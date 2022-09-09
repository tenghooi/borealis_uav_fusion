#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{

}

KalmanFilter::~KalmanFilter(){}

void KalmanFilter::reset()
{

}

void KalmanFilter::set_states(const Eigen::Vector3d& position, 
                              const Eigen::Vector3d& velocity,
                              const Eigen::Quaterniond& attitude)
{
    states_x_ << position, velocity, attitude;
}

void KalmanFilter::set_control_input(const Eigen::Vector3d& linear_accel,
                                     const Eigen::Vector3d& angular_vel)
{
    control_u_ << linear_accel, angular_vel;
}

void KalmanFilter::get_states(Eigen::VectorXd& states) const
{
    states = states_x_;
}

void KalmanFilter::PropagateState()
{
    states_x_ = state_transition_F_ * states_x_ + control_transition_B_ * control_u_;
}

void KalmanFilter::Predict()
{
    states_x_ = state_transition_F_ * states_x_ + control_transition_B_ * control_u_;
    states_cov_P_ = state_transition_F_ * states_cov_P_ * state_transition_F_.transpose()
                    + process_noise_cov_Q_;
}

void KalmanFilter::Update()
{
    Eigen::VectorXd measurement_residual_y;
    Eigen::MatrixXd residual_cov_S;
    
    measurement_residual_y = measurement_z_ - observation_transition_H_ * states_x_;

    residual_cov_S = observation_transition_H_ * states_cov_P_ * observation_transition_H_.transpose()
                     + measurement_noise_cov_R_;

    kalman_gain_K_ = states_cov_P_ * observation_transition_H_.transpose() * residual_cov_S.inverse();

    // Update states and its covariance
    states_x_ = states_x_ + kalman_gain_K_ * measurement_residual_y;

    states_cov_P_ = (identity_I_ - kalman_gain_K_ * observation_transition_H_) * states_cov_P_;
    

}