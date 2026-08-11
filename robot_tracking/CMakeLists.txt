cmake_minimum_required(VERSION 3.8)
project(robot_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(target_ekf src/target_ekf.cpp)
target_include_directories(target_ekf PRIVATE include)

ament_target_dependencies(target_ekf
  geometry_msgs
  nav_msgs
  rclcpp
)

target_link_libraries(target_ekf
  Eigen3::Eigen
)

install(TARGETS
  target_ekf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    constant_velocity_filter_test
    test/constant_velocity_filter_test.cpp
  )
  target_include_directories(constant_velocity_filter_test PRIVATE include)
  target_link_libraries(constant_velocity_filter_test Eigen3::Eigen)
endif()

ament_package()
