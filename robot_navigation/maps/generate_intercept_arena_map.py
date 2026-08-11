#!/usr/bin/env python3

from pathlib import Path


RESOLUTION = 0.1
WIDTH = 130
HEIGHT = 130
ORIGIN_X = -6.5
ORIGIN_Y = -6.5

# Conservative axis-aligned bounds for the four rotated arena crates.
OBSTACLES = (
    (3.8, 4.6, 3.2, 4.0),
    (-4.6, -3.4, 2.35, 3.25),
    (-3.95, -3.05, -4.6, -3.4),
    (3.3, 4.7, -3.8, -2.8),
)


def occupied(x, y):
    if x <= -6.0 or x >= 6.0 or y <= -6.0 or y >= 6.0:
        return True
    return any(
        min_x <= x <= max_x and min_y <= y <= max_y
        for min_x, max_x, min_y, max_y in OBSTACLES
    )


def main():
    pixels = bytearray()
    for row in range(HEIGHT):
        y = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION
        for column in range(WIDTH):
            x = ORIGIN_X + (column + 0.5) * RESOLUTION
            pixels.append(0 if occupied(x, y) else 254)

    output = Path(__file__).with_name("intercept_arena.pgm")
    header = f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii")
    output.write_bytes(header + pixels)


if __name__ == "__main__":
    main()
