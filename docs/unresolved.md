# Deferred Decisions

| Layer | Decision | Current default |
| --- | --- | --- |
| v0 tuning | Final capture/control gains | Start with 0.45 m capture radius, 1.0 m/s maximum speed, and repository defaults; tune only from recorded trials |
| v0 evaluation | Circular-model mismatch | Report position and velocity RMSE; do not hide lag by consuming target truth |
| v1 control | Nav2-to-terminal switch condition | Choose from measured distance, target-state age, and path feasibility after v0 data exists |
| v1 arbitration | Command ownership | Add a single explicit mux/owner; never rely on publisher timing |
| v2 localization | RGB-D odometry or monocular VIO | Prefer RGB-D for delivery robustness; choose monocular VIO only as a deliberate observability experiment |
| v2 resets | Discontinuity threshold and recovery | Define from localization error data before integration |
| v3 search | Forward-only search behavior | Deferred because v0 trials start with the target in view |
| v3 target model | Coordinated-turn filter | Add only after quantifying constant-velocity lag on the circular target |
