cmake_minimum_required(VERSION 3.8)
project(robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(color_detection src/color_detection.cpp)
add_executable(target_projector src/target_projector.cpp)

ament_target_dependencies(color_detection
  robot_interfaces
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

target_link_libraries(color_detection
  ${OpenCV_LIBRARIES}
)

target_link_libraries(target_projector
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(target_projector
  cv_bridge
  robot_interfaces
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  color_detection
  target_projector
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
