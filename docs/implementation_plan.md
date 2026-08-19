# Layered Implementation Plan

## v0: Direct Interception

The v0 runtime has four application nodes:

1. HSV target detection
2. RGB-D projection using synchronized ego ground-truth odometry
3. Constant-velocity Kalman filtering
4. Intercept solving and direct Ackermann pursuit control

Gazebo, bridges, robot state publication, the Ackermann controller, and the
optional evaluator are infrastructure rather than application nodes.

### Completion Gates

1. Projection error is measured against target truth across representative
   ranges and bearings.
2. The filter converges on synthetic constant-velocity data, rejects implausible
   innovations, and resets after long measurement gaps.
3. Intercept math covers stationary, receding, and unreachable targets.
4. Invalid or stale runtime inputs produce an active zero command.
5. At least 8 of 10 seeded trials capture within 0.45 m for 0.2 s before the
   20 s timeout.

Target truth must never be an input to perception, tracking, or control.

## v1: Mid-Course Obstacle Avoidance

The implemented v1 path adds two fixed chicane barriers and a matching static
map. Ground-truth ego odometry remains the localization input and publishes the
`map -> odom -> base_footprint` TF chain. Camera-derived
`/tracking/target_state` drives both the interception supervisor and the direct
controller; application nodes do not consume target truth.

Outside 1.0 m, the supervisor refreshes `NavigateToPose` intercept goals and
Nav2 routes around the static obstacles. Inside 1.0 m, direct terminal pursuit
takes over, with a 1.25 m exit threshold for hysteresis. The arbiter is the sole
publisher to `/cmd_vel`: `/navigation/cmd_vel_owner` uses `STOP=0`, `NAV2=1`,
and `TERMINAL=2` to select `/cmd_vel/nav2` or `/cmd_vel/terminal`.

Run with:

```bash
ros2 launch robot_sim v1_intercept.launch.py
ros2 run robot_sim run_v1_trials.py --trials 10 --required-successes 8
```

The proposed completion gate is at least 8 captures in the 10 deterministic
visible-target scenarios, zero fixed-obstacle contacts, complete collision data
for every trial, and bounded trial/process timeouts. The map/world/launch
contracts and pure helpers are statically tested. Actual ROS/Gazebo gate results
are not recorded until that environment is available and the command above is
run successfully.

## v2: Measured Ego Localization

Replace `/ground_truth/odom` at the existing projector and controller
interfaces. Decide deliberately between RGB-D odometry for robustness and
monocular VIO for the observability learning objective. Require bounded
position/yaw error and documented reset behavior before localization becomes
the default.

## v3: Resilience

Add explicit initializing, searching, tracking, intercepting, target-lost,
localization-lost, captured, and stopped states. Test detection dropout,
localization discontinuities, command timeout, and recovery behavior.
