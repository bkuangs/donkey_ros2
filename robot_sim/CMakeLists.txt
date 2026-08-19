cmake_minimum_required(VERSION 3.8)
project(robot_sim)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/cmd_vel_stamper.py
    scripts/run_trials.py
    scripts/run_v1_trials.py
    scripts/run_v2_trials.py
    scripts/trial_evaluator.py
    scripts/v1_obstacle_geometry.py
    scripts/v1_trial_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/trial_evaluation.py
    scripts/trial_process.py
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
  ament_add_pytest_test(
    v1_obstacles_test
    test/test_v1_obstacles.py
  )
  ament_add_pytest_test(
    trial_helpers_test
    test/test_trial_helpers.py
  )
  ament_add_pytest_test(
    v2_evaluation_test
    test/test_v2_evaluation.py
  )
endif()

ament_package()
