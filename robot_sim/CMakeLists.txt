cmake_minimum_required(VERSION 3.8)
project(robot_sim)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/cmd_vel_stamper.py
    scripts/run_trials.py
    scripts/trial_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models worlds urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    red_ball_model_test
    test/test_red_ball_model.py
  )
endif()

ament_package()
