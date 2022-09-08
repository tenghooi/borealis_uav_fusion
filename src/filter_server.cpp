#include "filter_server.h"

FilterServer::FilterServer(ros::NodeHandle node) 
{
    state_buffer_  = std::unique_ptr<std::array<State, N_STATE_BUFFER>>(new std::array<State, N_STATE_BUFFER>);

    imu_sub_ = node.subscribe("imu", 100, &FilterServer::IMUCallBack, this);
    pose_sub_ = node.subscribe("pose", 10, &FilterServer::PoseCallBack, this);

    fused_pose_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("uav_fusion_pose", 1);
}

FilterServer::~FilterServer()
{

}

void FilterServer::setNodeParams()
{

}

void FilterServer::setUpdateHandler()
{

}

void FilterServer::setStateBuffer(const uint16_t idx_state,
                    const sensor_msgs::ImuConstPtr& imu_msg)
{
    (*state_buffer_)[idx_state].setStateTime(imu_msg -> header);
    
    (*state_buffer_)[idx_state].linear_accel_imu_ << imu_msg -> linear_acceleration.x,
                                                  imu_msg -> linear_acceleration.y,
                                                  imu_msg -> linear_acceleration.z;

    (*state_buffer_)[idx_state].angular_vel_imu_ << imu_msg -> angular_velocity.x,
                                                 imu_msg -> angular_velocity.y,
                                                 imu_msg -> angular_velocity.z;
}

void FilterServer::toFilter(const uint16_t& idx_state)
                             
{
    uint16_t last_state = idx_state - 1;

    kalman_filter_.set_states((*state_buffer_)[last_state].position_,
                              (*state_buffer_)[last_state].velocity_,
                              (*state_buffer_)[last_state].attitude_);

    kalman_filter_.set_control_input((*state_buffer_)[idx_state].linear_accel_imu_,
                                     (*state_buffer_)[idx_state].angular_vel_imu_);
    
}

void FilterServer::fromFilter(const uint16_t& idx_state)
{
    Eigen::VectorXd states(N_STATES);
    kalman_filter_.get_states(states);

    (*state_buffer_)[idx_state].velocity_ = states.segment(3,5);
    (*state_buffer_)[idx_state].attitude_.vec() = states.segment(6,8);
    (*state_buffer_)[idx_state].attitude_.w() = states[9];
    (*state_buffer_)[idx_state].position_ = states.segment(0,2);

}

void FilterServer::StatePropagationProcess(const uint16_t& idx_state,
                                           const sensor_msgs::ImuConstPtr& imu_msg)
{   
    if (update_handler_.IsImuMsg())
    {   
        setStateBuffer(idx_state, imu_msg);
        toFilter(idx_state);

        kalman_filter_.PropagateState();

        fromFilter(idx_state);

        geometry_msgs::PoseWithCovarianceStamped new_pose;
        (*state_buffer_)[idx_state].ConvertToPoseMsg(new_pose);
        fused_pose_pub_.publish(new_pose);

        ROS_INFO_STREAM("State propagated with new IMU msg");
    }
    else
    {
        ROS_WARN_STREAM("Not IMU msg! State not propagated.");
    }
    
}

void FilterServer::StateUpdateProcess(const uint16_t& idx_measurement,
                                      const nav_msgs::OdometryConstPtr& measurement_msg)
{

}

void FilterServer::IMUCallBack(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double msg_time = imu_msg->header.stamp.toSec();

    uint16_t idx_state = update_handler_.getCurrentStateIdx();

    if (msg_time > (*state_buffer_)[idx_state].state_time_)
    {   
        bool is_imu_msg = true;
        update_handler_.setIsImuMsg(is_imu_msg);

        idx_state += 1;
        StatePropagationProcess(idx_state, imu_msg);

        update_handler_.setCurrentStateIdx(idx_state);

        is_imu_msg = false;
        update_handler_.setIsImuMsg(is_imu_msg);
    }
    else
    {
        ROS_WARN_STREAM("IMU msg time invalid. Too old!");
    }
    
}

void FilterServer::PoseCallBack(const nav_msgs::OdometryConstPtr& measurement_msg)
{
    uint16_t idx_measurement = update_handler_.getLastMeaStateIdx();

    StateUpdateProcess(idx_measurement, measurement_msg);
}

