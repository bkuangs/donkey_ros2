## Monocular Visual-Inertial Odometry (VIO) & ROS 2 Nav2

### Project:
Add a cheap IMU to the Donkey Car. Strip the standard software and rebuild the architecture entirely in ROS 2. Implement a lightweight SLAM or VIO package (like RTAB-Map or a scaled-down ORB-SLAM) to fuse camera and IMU data. Use the ROS 2 Nav2 stack to command the vehicle to drive to arbitrary coordinates in its mapped environment.

**Tech Stack**: C++, ROS 2 (Humble/Jazzy), OpenCV, Nav2.

Building a full ROS 2 hardware architecture from scratch proves you aren't just running tutorials. VIO and state estimation are notoriously difficult on constrained hardware; successfully implementing them shows serious systems engineering chops.

## Dynamic Target Intercept System

### Project: 
Program the car to identify, lock onto, and physically intercept a moving target (e.g., another RC car, a rolling ball, or a person walking). Use classical computer vision (optical flow) or a lightweight object detector to find the target. Pass the bounding box data into an Extended Kalman Filter (EKF) to estimate the target's velocity and trajectory. Finally, use a pure pursuit algorithm to generate steering commands to intercept it.

**Tech Stack**: C++, ROS 2, OpenCV, Kalman Filters.

This perfectly mimics the sensor-fusion-to-actuation loop used in autonomous defense systems. It highlights applied linear algebra, predictive modeling, and real-time control.
