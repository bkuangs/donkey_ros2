cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(intercept_guidance src/intercept_guidance.cpp)
add_executable(nav2_goal_manager src/nav2_goal_manager.cpp)
ament_target_dependencies(intercept_guidance
  rclcpp
  nav_msgs
  geometry_msgs
)
ament_target_dependencies(nav2_goal_manager
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
)

install(TARGETS intercept_guidance nav2_goal_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
