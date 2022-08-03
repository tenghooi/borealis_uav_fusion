# borealis_uav_fusion
A ROS package for UAV states sensor fusion based on EKF for Borealis project. 

This is an implementation of ROS [ethzasl_sensor_fusion](http://wiki.ros.org/ethzasl_sensor_fusion) package based on the following paper.

[Paper](https://ieeexplore.ieee.org/document/6225002):
S. Weiss, M. W. Achtelik, M. Chli and R. Siegwart, *Versatile distributed pose estimation and sensor self-calibration for an autonomous MAV*, IEEE International Conference on Robotics and Automation, 2012, pp. 31-38.

### Sensors deployed:
In our case, we used a Ouster OS1-32 Gen2 as the sensor for state estimator and its built-in IMU readings as our control input.

- 3D Lidar
- IMU

Theoretically it works with any sensor that estimates 6-DOF states together with an IMU with proper defined rigid transformation matrix between them.

