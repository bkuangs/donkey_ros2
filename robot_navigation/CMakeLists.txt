cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(intercept_controller src/intercept_controller.cpp)
target_include_directories(intercept_controller PRIVATE include)
ament_target_dependencies(intercept_controller
  rclcpp
  nav_msgs
  geometry_msgs
)

add_executable(command_arbiter src/command_arbiter.cpp)
target_include_directories(command_arbiter PRIVATE include)
ament_target_dependencies(command_arbiter
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(interception_supervisor src/interception_supervisor.cpp)
target_include_directories(interception_supervisor PRIVATE include)
ament_target_dependencies(interception_supervisor
  rclcpp
  rclcpp_action
  action_msgs
  nav_msgs
  nav2_msgs
  std_msgs
)

install(TARGETS intercept_controller command_arbiter interception_supervisor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(intercept_math_test test/intercept_math_test.cpp)
  target_include_directories(intercept_math_test PRIVATE include)

  ament_add_gtest(control_logic_test test/control_logic_test.cpp)
  target_include_directories(control_logic_test PRIVATE include)
endif()

ament_package()
