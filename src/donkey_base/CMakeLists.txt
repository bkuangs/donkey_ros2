cmake_minimum_required(VERSION 3.8)
project(donkey_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(donkey_base_node src/donkey_base_node.cpp)
ament_target_dependencies(
  donkey_base_node
  rclcpp
  std_msgs
  ackermann_msgs
  diagnostic_msgs
)

install(TARGETS donkey_base_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
