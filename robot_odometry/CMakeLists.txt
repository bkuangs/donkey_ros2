cmake_minimum_required(VERSION 3.8)
project(robot_odometry)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    robot_odometry_assets
    test/test_assets.py
  )
endif()

ament_package()
