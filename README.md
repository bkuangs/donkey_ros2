## Dynamic Target Intercept using Monocular Visual-Inertial Odometry

### Description:
Using IMU and camera, implement a lightweight SLAM or VIO package (like RTAB-Map or a scaled-down ORB-SLAM) to fuse camera and IMU data. Use the ROS 2 Nav2 stack to command the vehicle to drive to arbitrary coordinates in its mapped environment.

Program the car to identify, lock onto, and physically intercept a moving target (e.g., another RC car, a rolling ball, or a person walking). Use classical computer vision (optical flow) or a lightweight object detector to find the target. Pass the bounding box data into an Extended Kalman Filter (EKF) to estimate the target's velocity and trajectory. Finally, use a pure pursuit algorithm to generate steering commands to intercept it.

**Stack**: C++, ROS 2, OpenCV, Nav2, EKF  

**Concepts:** Sensor fusion, applied linear algebra, predictive modeling, real-time control

### Architecture:
- _localization: VIO, EKF, odometry, TF
- _perception: Target detection from images (OpenCV)
- _tracking: Target state estimatation (EKF)
- _control: Pure pursuit, PID controller
- _navigation: Decides where the vehicle should go (Goal generation? Unclear for now)