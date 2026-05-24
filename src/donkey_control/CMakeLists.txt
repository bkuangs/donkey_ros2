cmake_minimum_required(VERSION 3.8)
project(donkey_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(command_mux src/command_mux.cpp)
ament_target_dependencies(command_mux
  rclcpp
  ackermann_msgs
)

add_executable(twist_to_ack src/twist_to_ack.cpp)
ament_target_dependencies(twist_to_ack
  rclcpp
  geometry_msgs
  ackermann_msgs
)

add_executable(ack_to_joint_cmd src/ack_to_joint_cmd.cpp)
ament_target_dependencies(ack_to_joint_cmd
  rclcpp
  std_msgs
  ackermann_msgs
)

install(TARGETS
  command_mux
  twist_to_ack
  ack_to_joint_cmd
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
