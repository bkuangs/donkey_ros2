cmake_minimum_required(VERSION 3.8)
project(donkey_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
