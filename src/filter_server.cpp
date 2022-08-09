#include "filter_server.h"

FilterServer::FilterServer(ros::NodeHandle node) 
{

}

FilterServer::~FilterServer()
{

}

void FilterServer::SetNodeParams()
{

}

void FilterServer::SetUpdateHandler()
{

}

void FilterServer::StatePropagationProcess()
{

}

void FilterServer::StateUpdateProcess()
{

}

void FilterServer::PoseCallBack(const nav_msgs::OdometryConstPtr& measurement_msg)
{
    uint16_t idx_state = update_handler_.getCurrentStateIdx();

    StatePropagationProcess();
}

void FilterServer::IMUCallBack(const sensor_msgs::ImuConstPtr& imu_msg)
{

}