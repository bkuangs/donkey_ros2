cmake_minimum_required(VERSION 3.8)
project(robot_sim)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/cmd_vel_stamper.py
    scripts/run_trials.py
    scripts/trial_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models worlds urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
