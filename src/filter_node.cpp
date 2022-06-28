#include <iostream>

#include <ros/ros.h>                   
                   
int main(int argc, char** argv)
{           
    ros::init(argc, argv, "uav_fusion");
    ros::NodeHandle node("~");

    ros::spin();

    return 0;
}