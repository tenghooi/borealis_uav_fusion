#ifndef _BOREALIS_FILTER_SERVER_H_
#define _BOREALIS_FILTER_SERVER_H_

#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <queue>
#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include "node_param.h"
#include "state.h"
#include "update_handler.h"
#include "kalman_filter.h"
#include "ekf.h"

#define N_STATE_BUFFER 65536 // size of uint16_t, DO NOT change

class FilterServer
{
private:
    
    //std::vector<State>state_buffer_;
    std::unique_ptr<std::array<State, N_STATE_BUFFER>> state_buffer_; // ring buffer for uav's states

    NodeParams parameters_;

    KalmanFilter kalman_filter_;

    UpdateHandler update_handler_;

    ros::Subscriber pose_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher fused_pose_pub_;

public:
    FilterServer(ros::NodeHandle node);
    ~FilterServer();

    void Initialize();

    void setNodeParams();
    void setUpdateHandler();
    void setStateBuffer(const uint16_t idx_state, const sensor_msgs::ImuConstPtr& imu_msg);
    
    void toFilter(const uint16_t& idx_state);
    void fromFilter(const uint16_t& idx_state);
    
    void StatePropagationProcess(const uint16_t& idx_state, const sensor_msgs::ImuConstPtr& imu_msg);
    void StateUpdateProcess(const uint16_t& idx_measurement, const nav_msgs::OdometryConstPtr& measurement_msg);

    void IMUCallBack(const sensor_msgs::ImuConstPtr& imu_msg);
    void PoseCallBack(const nav_msgs::OdometryConstPtr& measurement_msg);
};

#endif // _BOREALIS_FILTER_SERVER_H_