#include <iostream>

#include <ros/ros.h>     

#include "filter_server.h"
                   
int main(int argc, char** argv)
{           
    std::cout << "gg" <<std::endl;
    ros::init(argc, argv, "uav_fusion");
    ros::NodeHandle node("~");
    std::cout << "gg1" <<std::endl;
    FilterServer filter_server(node);

    ros::spin();

    return 0;
}