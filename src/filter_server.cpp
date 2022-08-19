#include "filter_server.h"

FilterServer::FilterServer(ros::NodeHandle node) 
{

}

FilterServer::~FilterServer()
{

}

void FilterServer::setIMUBuffer(uint16_t idx, const sensor_msgs::ImuConstPtr& imu_msg)
{

}

void FilterServer::setNodeParams()
{

}

void FilterServer::setUpdateHandler()
{

}

void FilterServer::StatePropagationProcess(const uint16_t& idx_state)
{   
    if (update_handler_.IsImuMsg())
    {   


        update_handler_.setCurrentStateIdx(idx_state);
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

void FilterServer::PoseCallBack(const nav_msgs::OdometryConstPtr& measurement_msg)
{
    uint16_t idx_measurement = update_handler_.getLastMeaStateIdx();

    StateUpdateProcess(idx_measurement, measurement_msg);
}

void FilterServer::IMUCallBack(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double msg_time = imu_msg->header.stamp.toSec();

    uint16_t idx_state = update_handler_.getCurrentStateIdx();

    if (msg_time > state_buffer_[idx_state].state_time_)
    {   
        bool is_imu_msg = true;
        update_handler_.setIsImuMsg(is_imu_msg);

        idx_state += 1;
        StatePropagationProcess(idx_state);

        is_imu_msg = false;
        update_handler_.setIsImuMsg(is_imu_msg);
    }
    else
    {
        ROS_WARN_STREAM("IMU msg time invalid. Too old!");
    }
    
}