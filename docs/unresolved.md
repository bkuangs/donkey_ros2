# Deferred Decisions

| Layer | Decision | Current default |
| --- | --- | --- |
| v0 tuning | Final capture/control gains | Start with 0.45 m capture radius, 1.0 m/s maximum speed, and repository defaults; tune only from recorded trials |
| v0 evaluation | Circular-model mismatch | Report position and velocity RMSE; do not hide lag by consuming target truth |
| v1 runtime validation | Completion gate result | Implemented gate requires 8 of 10 deterministic captures, zero fixed-obstacle contacts, complete collision data, and bounded timeouts; no ROS/Gazebo result is recorded yet |
| v2 runtime validation | Completion gate result | RGB-D odometry is implemented; run the 10 deterministic Jazzy/Gazebo trials and record capture, collision, availability, and localization-error results |
| v2 resets | In-process recovery | Trials fail closed on timestamp regression and use a fresh process; autonomous recovery remains deferred to v3 |
| v3 search | Forward-only search behavior | Deferred because v0 trials start with the target in view |
| v3 target model | Coordinated-turn filter | Add only after quantifying constant-velocity lag on the circular target |
