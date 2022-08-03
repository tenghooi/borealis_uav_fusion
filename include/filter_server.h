#ifndef _BOREALIS_FILTER_SERVER_H_
#define _BOREALIS_FILTER_SERVER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>

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


class FilterServer
{
private:
    NodeParams parameters_;

    KalmanFilter kalman_filter_;

    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher fused_pose_pub_;

public:
    FilterServer(ros::NodeHandle node);
    ~FilterServer();

    void PoseCallBack(const nav_msgs::OdometryConstPtr& measurement_msg);
    void ControlCallBack(const sensor_msgs::ImuConstPtr& control_msg);
};

#endif // _BOREALIS_FILTER_SERVER_H_