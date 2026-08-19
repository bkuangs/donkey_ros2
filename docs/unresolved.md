# Deferred Decisions

| Layer | Decision | Current default |
| --- | --- | --- |
| v0 tuning | Final capture/control gains | Start with 0.45 m capture radius, 1.0 m/s maximum speed, and repository defaults; tune only from recorded trials |
| v0 evaluation | Circular-model mismatch | Report position and velocity RMSE; do not hide lag by consuming target truth |
| v1 runtime validation | Completion gate result | Passed on Jazzy with 10 of 10 captures and no reported obstacle contacts |
| v2 runtime validation | Completion gate result | The 2026-08-19 Jazzy/QEMU run captured 8 of 10 with zero observed contacts, but failed completeness and localization thresholds; isolate raw wheel and visual odometry before tuning fusion |
| v2 resets | In-process recovery | Trials fail closed on timestamp regression and use a fresh process; autonomous recovery remains deferred to v3 |
| v3 search | Forward-only search behavior | Deferred because v0 trials start with the target in view |
| v3 target model | Coordinated-turn filter | Add only after quantifying constant-velocity lag on the circular target |
