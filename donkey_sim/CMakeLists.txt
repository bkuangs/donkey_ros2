cmake_minimum_required(VERSION 3.8)
project(donkey_sim)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/cmd_vel_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config worlds urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
