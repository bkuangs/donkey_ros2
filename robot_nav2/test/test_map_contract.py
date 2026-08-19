import json
import math
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def read_pgm(path):
    with path.open("rb") as stream:
        assert stream.readline().strip() == b"P5"
        width, height = (int(value) for value in stream.readline().split())
        assert int(stream.readline()) == 255
        pixels = stream.read()
    assert len(pixels) == width * height
    return width, height, pixels


def test_map_metadata_matches_arena_contract():
    contract = json.loads((PACKAGE_ROOT / "maps" / "arena_contract.json").read_text())
    yaml_lines = (PACKAGE_ROOT / "maps" / "arena.yaml").read_text().splitlines()
    metadata = dict(line.split(":", 1) for line in yaml_lines if ":" in line)
    assert metadata["image"].strip() == "arena.pgm"
    assert float(metadata["resolution"]) == contract["resolution_m"]
    assert metadata["origin"].strip() == "[-6.0, -6.0, 0.0]"

    width, height, _ = read_pgm(PACKAGE_ROOT / "maps" / "arena.pgm")
    assert (width, height) == (contract["width_cells"], contract["height_cells"])
    assert width * contract["resolution_m"] == 12.0
    assert height * contract["resolution_m"] == 12.0


def test_obstacle_rectangles_and_protected_regions_are_aligned():
    contract = json.loads((PACKAGE_ROOT / "maps" / "arena_contract.json").read_text())
    _, _, pixels = read_pgm(PACKAGE_ROOT / "maps" / "arena.pgm")
    resolution = contract["resolution_m"]
    origin_x, origin_y = contract["origin_xy_m"]
    epsilon = 1.0e-9

    for row in range(contract["height_cells"]):
        y = origin_y + (row + 0.5) * resolution
        for column in range(contract["width_cells"]):
            x = origin_x + (column + 0.5) * resolution
            in_boundary = (
                x < -5.9 or x >= 5.9 or y < -5.9 or y >= 5.9
            )
            in_obstacle = any(
                obstacle["x_m"][0] < x + resolution / 2.0 - epsilon
                and x - resolution / 2.0 < obstacle["x_m"][1] - epsilon
                and obstacle["y_m"][0] < y + resolution / 2.0 - epsilon
                and y - resolution / 2.0 < obstacle["y_m"][1] - epsilon
                for obstacle in contract["fixed_obstacles"]
            )
            expected = 0 if in_boundary or in_obstacle else 254
            image_row = contract["height_cells"] - 1 - row
            actual = pixels[image_row * contract["width_cells"] + column]
            assert actual == expected

            radius = math.hypot(x, y)
            if -2.5 <= x <= -2.3 and -0.5 <= y <= 0.5:
                assert expected == 254
            if 2.0 <= radius <= 3.35:
                assert expected == 254
