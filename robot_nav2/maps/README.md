# Arena map contract

`arena.pgm` is a 240 x 240 trinary map at 0.05 m/cell. Its lower-left
corner is `(-6, -6)` in `map`; `map` and simulator `odom` are identical.
The outermost 0.10 m is occupied to represent the inner faces of the arena
walls.

The fixed obstacles are axis-aligned, half-open rectangles:

| name | x range (m) | y range (m) | SDF center (m) | SDF size (m) |
|---|---:|---:|---:|---:|
| chicane_barrier_a | [-1.025, -0.775) | [-1.2, 1.2) | (-0.9, 0.0, 0.06) | (0.25, 2.4, 0.12) |
| chicane_barrier_b | [0.775, 1.025) | [-1.2, 1.2) | (0.9, 0.0, 0.06) | (0.25, 2.4, 0.12) |

These barriers force a north/south detour from the v1 start region at
`x=[-2.5,-2.3]`, `y=[-0.5,0.5]`. Their outer radius is 1.578 m. At the 1.0 m
terminal-entry distance from the target's 3 m orbit, the robot center remains
at radius 2.0 m or greater; subtracting its 0.216 m footprint radius still
leaves it outside the barriers. Rasterization is conservative: every map cell
with positive-area overlap with an obstacle is occupied.
The complete machine-readable contract is `arena_contract.json`. The map
intentionally contains static geometry only; target truth is never an
obstacle/perception source.
