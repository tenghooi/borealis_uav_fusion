#include <iostream>

#include <ros/ros.h>     

#include "filter_server.h"
                   
int main(int argc, char** argv)
{           
    ros::init(argc, argv, "uav_fusion");
    ros::NodeHandle node("~");
 
    FilterServer filter_server(node);

    ros::spin();

    return 0;
}