# Target Interception Simulation

This ROS 2 project demonstrates the estimation and control required for an
Ackermann vehicle to intercept a moving target. The v0 system deliberately uses
simulator ground truth for ego pose so that target projection, filtering,
intercept prediction, and terminal guidance can be validated without a
localization or navigation stack in the critical path.

## v0 Scope

The application consists of four nodes:

```text
/camera/image_raw
  -> color_detection
  -> target_projector <- /camera/depth_image + /ground_truth/odom
  -> target_ekf
  -> intercept_controller <- /ground_truth/odom
  -> /cmd_vel
```

| Package | Responsibility |
| --- | --- |
| `robot_description` | Ackermann vehicle geometry and static sensor extrinsics |
| `robot_sim` | Gazebo arena, sensors, bridges, control, and trial evaluation |
| `robot_interfaces` | Stamped 2D target-detection message |
| `robot_perception` | HSV detection and synchronized RGB-D projection |
| `robot_tracking` | Constant-velocity target Kalman filter |
| `robot_navigation` | Intercept solve and direct Ackermann pursuit control |

OpenVINS, Nav2, lifecycle managers, behavior trees, known maps, costmaps, and
obstacles are intentionally outside v0.

## Scenario

The vehicle starts inside a 12 m obstacle-free arena. A red ball moves at
0.4 m/s on a 3 m circle. The RGB-D pipeline estimates the target in `odom`, and
the controller repeatedly solves

```text
|p_target - p_robot + v_target t| = v_robot t
```

before steering toward the resulting intercept point. The constant-velocity
filter is intentionally model-mismatched with circular motion; its lag is a
measured limitation rather than hidden by using target truth.

`/ground_truth/odom` is an intentional v0 input. Target truth at
`/target/ground_truth/odom` is consumed only by the optional evaluator.

## Build and Run

Use a ROS 2 Jazzy workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_sim intercept.launch.py
```

The default target starts at `(3, 0)` with the vehicle facing it. Robot and
target initial conditions are launch arguments:

```bash
ros2 launch robot_sim intercept.launch.py \
  robot_x:=0.5 robot_y:=-0.25 robot_yaw:=0.1 \
  target_x:=3.0 target_y:=0.0 target_yaw:=1.5708
```

## Acceptance Trials

Run the seeded ten-trial gate after building:

```bash
ros2 run robot_sim run_trials.py \
  --trials 10 \
  --required-successes 8 \
  --output-dir trial_results
```

Each trial randomizes the target phase and the vehicle pose while initially
aiming the forward camera toward the target. `trial_results/summary.json`
contains each seed, minimum center distance, capture time, target-estimation
RMSE, and termination reason.

The v0 gate is:

| Criterion | Value |
| --- | --- |
| Capture radius | 0.45 m center-to-center |
| Capture dwell | 0.2 s |
| Trial timeout | 20 s |
| Required successes | 8 of 10 |
| Stale ego or target input | Zero command within 0.25 s |

## Safety and Control Limits

The controller is forward-only and assumes the target is initially visible. It
actively publishes a zero command when input is stale, the target is captured,
or no intercept exists within the configured horizon. Curvature is limited by
the 0.24 m wheelbase and 0.6 rad steering limit; linear acceleration and
deceleration are also bounded.

## Roadmap

| Version | Increment |
| --- | --- |
| v0 | Open-space interception using ego ground truth |
| v1 | Obstacles and Nav2 for mid-course routing, with direct terminal pursuit |
| v2 | Replace ego truth with measured odometry or deliberately chosen VIO |
| v3 | Search, loss recovery, reset handling, and explicit mission states |

See [`docs/implementation_plan.md`](docs/implementation_plan.md) for gates
between layers and [`docs/unresolved.md`](docs/unresolved.md) for deferred
decisions.
