#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{

}

KalmanFilter::~KalmanFilter(){}

void KalmanFilter::reset()
{
    // state_transition_F_.Zero();
    // control_transition_B_.Zero();
    // observation_transition_H_.Zero();

    // states_x_.Zero();
    // control_u_.Zero();
    // measurement_z_.Zero();

    // states_cov_P_.Zero();
    // process_noise_cov_Q_.Zero();
    // measurement_noise_cov_R_.Zero();

    // identity_I_.Identity();
    // kalman_gain_K_.Zero();
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

Eigen::MatrixXd KalmanFilter::get_B() const
{
    return control_transition_B_;
}

void KalmanFilter::PropagatePositionAndVelocity()
{

}

void KalmanFilter::PropagateQuaternion(double angular_vel_mag)
{
    double alpha;
    double q_w, q_x, q_y, q_z;
    double w_x, w_y, w_z; 

    alpha = angular_vel_mag * 0.01 / 2.0;

    q_x = states_x_[6];
    q_y = states_x_[7];
    q_z = states_x_[8];
    q_w = states_x_[9];

    w_x = control_u_[3];
    w_y = control_u_[4];
    w_z = control_u_[5];

    states_x_[9] = q_w * cos(alpha) - 
                   (q_x * w_x + q_y * w_y + q_z * w_z) * sin(alpha) / angular_vel_mag; // w - component of quaternion
    
    states_x_[6] = states_x_[6]; // x - component of quaternion
    
    states_x_[7] = states_x_[6]; // y - component of quaternion
    
    states_x_[8] = states_x_[6]; // z - component of quaternion
}

void KalmanFilter::PropagateState()
{   
    double angular_vel_mag;
    angular_vel_mag = control_u_.segment(3,5).norm();

    PropagateQuaternion(angular_vel_mag);
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