cmake_minimum_required(VERSION 3.8)
project(donkey_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(pure_pursuit src/pure_pursuit.cpp)

ament_target_dependencies(pure_pursuit
  rclcpp
  nav_msgs
  ackermann_msgs
)

install(TARGETS
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()