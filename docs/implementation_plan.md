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

The completion gate is at least 8 captures in the 10 deterministic
visible-target scenarios, zero fixed-obstacle contacts, complete collision data
for every trial, and bounded trial/process timeouts. The map/world/launch
contracts and pure helpers are statically tested. The Jazzy gate completed with
10 captures and no reported obstacle contacts.

## v2: Measured Ego Localization

The implemented v2 path uses `rtabmap_odom/rgbd_odometry` with the existing RGB,
depth, and camera-info streams. A planar `robot_localization` EKF combines its
x/y/yaw pose with encoder-derived wheel velocity, then publishes
`/localization/odom` and `odom -> base_footprint`. A launch-time static
`map -> odom` transform seeds the known initial robot pose. This avoids runtime
ego truth without adding the IMU and reset scope required by monocular VIO.

The v2 launch routes `/localization/odom` to target projection, Nav2, the
interception supervisor, and terminal control, while disabling the v1
ground-truth TF adapter. Ground-truth ego and target topics remain available
only to the trial evaluator.

Run with:

```bash
ros2 launch robot_sim v2_intercept.launch.py
ros2 run robot_sim run_v2_trials.py --trials 10 --required-successes 8
```

The gate reuses the ten deterministic v1 scenarios and requires at least eight
captures, zero obstacle contacts, complete collision/localization data, position
RMSE no greater than 0.20 m, yaw RMSE no greater than 0.15 rad, final position
error no greater than 0.30 m, and localization availability of at least 95% in
every trial. Trials use fresh processes; timestamp regressions and missing
localization fail closed.

The 2026-08-19 Jazzy/QEMU gate captured 8 of 10 scenarios with zero observed
contacts, but did not pass. One trial timed out, one hit the process timeout, and
the completed runs exposed inconsistent fused localization: position RMSE
reached 0.403 m, yaw RMSE reached 0.324 rad, final position error reached
0.650 m, and minimum availability was 91.8%. Raw wheel and raw visual odometry
should be evaluated independently before further fusion tuning.

## v3: Resilience

Add explicit initializing, searching, tracking, intercepting, target-lost,
localization-lost, captured, and stopped states. Test detection dropout,
localization discontinuities, command timeout, and recovery behavior.
