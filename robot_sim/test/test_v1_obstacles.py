import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from run_v1_trials import V1_SCENARIOS, evaluate_gate, scenario_geometry
from v1_obstacle_geometry import V1_OBSTACLES, obstacle_clearances


V1_WORLD_PATH = PACKAGE_ROOT / "worlds" / "intercept_arena_v1.sdf"
V0_WORLD_PATH = PACKAGE_ROOT / "worlds" / "intercept_arena.sdf"
MAP_CONTRACT_PATH = (
    PACKAGE_ROOT.parent / "robot_nav2" / "maps" / "arena_contract.json"
)


def _vector(text):
    return tuple(float(value) for value in text.split())


def test_v1_world_obstacles_match_evaluator_contract():
    world = ET.parse(V1_WORLD_PATH).getroot().find("world")
    assert world.attrib["name"] == "intercept_arena_v1"

    for obstacle in V1_OBSTACLES:
        model = world.find(f"model[@name='{obstacle.name}']")
        assert model is not None
        assert model.findtext("static") == "true"
        pose = _vector(model.findtext("pose"))
        expected_pose = (
            obstacle.center_x,
            obstacle.center_y,
            obstacle.height / 2.0,
            0.0,
            0.0,
            0.0,
        )
        assert pose == expected_pose

        link = model.find("link[@name='barrier']")
        expected_size = (
            obstacle.size_x,
            obstacle.size_y,
            obstacle.height,
        )
        assert _vector(link.findtext("collision/geometry/box/size")) == expected_size
        assert _vector(link.findtext("visual/geometry/box/size")) == expected_size


def test_nav2_map_contract_matches_v1_obstacles():
    contract = json.loads(MAP_CONTRACT_PATH.read_text())
    by_name = {
        obstacle["name"]: obstacle for obstacle in contract["fixed_obstacles"]
    }
    assert set(by_name) == {obstacle.name for obstacle in V1_OBSTACLES}

    for obstacle in V1_OBSTACLES:
        mapped = by_name[obstacle.name]
        assert mapped["x_m"] == [
            obstacle.center_x - obstacle.size_x / 2.0,
            obstacle.center_x + obstacle.size_x / 2.0,
        ]
        assert mapped["y_m"] == [
            obstacle.center_y - obstacle.size_y / 2.0,
            obstacle.center_y + obstacle.size_y / 2.0,
        ]
        assert mapped["sdf_center_xyz_m"] == [
            obstacle.center_x,
            obstacle.center_y,
            obstacle.height / 2.0,
        ]
        assert mapped["sdf_size_xyz_m"] == [
            obstacle.size_x,
            obstacle.size_y,
            obstacle.height,
        ]


def test_v0_world_has_no_v1_obstacles():
    world = ET.parse(V0_WORLD_PATH).getroot().find("world")
    assert all(
        world.find(f"model[@name='{obstacle.name}']") is None
        for obstacle in V1_OBSTACLES
    )


def test_target_terminal_corridor_is_clear():
    outer_obstacle_radius = max(
        math.hypot(x, y)
        for obstacle in V1_OBSTACLES
        for x, y in obstacle.corners
    )
    target_orbit_radius = 3.0
    terminal_enter_distance = 1.0
    footprint_radius = math.hypot(0.36 / 2.0, 0.24 / 2.0)
    minimum_footprint_radius = (
        target_orbit_radius - terminal_enter_distance - footprint_radius
    )
    assert math.isclose(outer_obstacle_radius, math.hypot(1.025, 1.2))
    assert minimum_footprint_radius > outer_obstacle_radius


def test_clearance_detects_collision_touch_and_free_space():
    collision = obstacle_clearances(-0.9, 0.0, 0.0)
    assert collision["chicane_barrier_a"] == 0.0

    touching = obstacle_clearances(-0.9, 1.32, 0.0)
    assert math.isclose(touching["chicane_barrier_a"], 0.0, abs_tol=1e-12)

    between = obstacle_clearances(0.0, 0.0, 0.0)
    assert math.isclose(
        between["chicane_barrier_a"], 0.595, abs_tol=1e-12
    )
    assert math.isclose(
        between["chicane_barrier_b"], 0.595, abs_tol=1e-12
    )


def test_rotated_footprint_collision_is_conservative():
    clearances = obstacle_clearances(-0.9, 1.38, math.pi / 4.0)
    assert clearances["chicane_barrier_a"] == 0.0


def test_v1_scenarios_are_visible_feasible_and_deterministic():
    assert len(V1_SCENARIOS) == 10
    assert len({scenario[1] for scenario in V1_SCENARIOS}) == 10
    for scenario in V1_SCENARIOS:
        geometry = scenario_geometry(scenario)
        target_radius = math.hypot(
            geometry["target_x"], geometry["target_y"]
        )
        expected_bearing = math.atan2(
            geometry["target_y"] - geometry["robot_y"],
            geometry["target_x"] - geometry["robot_x"],
        )
        assert math.isclose(target_radius, 3.0)
        assert math.isclose(geometry["robot_yaw"], expected_bearing)
        assert min(
            obstacle_clearances(
                geometry["robot_x"],
                geometry["robot_y"],
                geometry["robot_yaw"],
            ).values()
        ) > 0.0
        for obstacle in V1_OBSTACLES:
            progress = (
                obstacle.center_x - geometry["robot_x"]
            ) / (geometry["target_x"] - geometry["robot_x"])
            direct_path_y = geometry["robot_y"] + progress * (
                geometry["target_y"] - geometry["robot_y"]
            )
            assert 0.0 < progress < 1.0
            assert abs(direct_path_y - obstacle.center_y) < obstacle.size_y / 2.0


def test_v1_gate_requires_eight_captures_and_zero_contacts():
    results = [
        {
            "captured": index < 8,
            "obstacle_contact_count": 0,
            "collision_data_complete": True,
        }
        for index in range(10)
    ]
    assert evaluate_gate(results, 8) == (8, 0, True, True)

    results[-1]["collision_data_complete"] = False
    assert evaluate_gate(results, 8) == (8, 0, False, False)

    results[-1]["obstacle_contact_count"] = 1
    assert evaluate_gate(results, 8) == (8, 1, False, False)

    results[7]["captured"] = False
    results[-1]["collision_data_complete"] = True
    assert evaluate_gate(results, 8) == (7, 1, True, False)
