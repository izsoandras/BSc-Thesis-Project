# BSc-Thesis-Project
#### Autonomous robotic platform navigation with embedded computer vision
During this project I designed and implemented a robotic platform and compared a few localization algorithms using it. These algorithms were:
- Magnetometer + Accelerometer
- Magnetometer + Encoder
- VISO2
- ORB-SLAM2

I modified the two computer vision library in order to compile and run it on a Raspberry 3b+. The forks of their repositories can be found here:
https://github.com/izsoandras/viso2

https://github.com/izsoandras/ORB_SLAM2

The robot uses the following ROS package for accessing the camera:
https://github.com/UbiquityRobotics/raspicam_node

I followed this tutorial to get the VISO2 working:
https://www.instructables.com/id/ROS-Visual-Odometry/
