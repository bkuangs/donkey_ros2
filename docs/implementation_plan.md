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

Add obstacles and Nav2 only after v0 passes. Nav2 owns mid-course routing while
the direct controller owns terminal interception inside a configured switch
radius. Add explicit command arbitration so Nav2 and terminal pursuit can never
publish competing commands.

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
