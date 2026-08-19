#!/usr/bin/env python3

import math
from dataclasses import dataclass


ROBOT_FOOTPRINT_LENGTH = 0.36
ROBOT_FOOTPRINT_WIDTH = 0.24


@dataclass(frozen=True)
class Rectangle:
    name: str
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    height: float

    @property
    def corners(self):
        half_x = self.size_x / 2.0
        half_y = self.size_y / 2.0
        return (
            (self.center_x - half_x, self.center_y - half_y),
            (self.center_x + half_x, self.center_y - half_y),
            (self.center_x + half_x, self.center_y + half_y),
            (self.center_x - half_x, self.center_y + half_y),
        )


V1_OBSTACLES = (
    Rectangle("chicane_barrier_a", -0.9, 0.0, 0.25, 2.4, 0.12),
    Rectangle("chicane_barrier_b", 0.9, 0.0, 0.25, 2.4, 0.12),
)


def footprint_corners(
    x,
    y,
    yaw,
    length=ROBOT_FOOTPRINT_LENGTH,
    width=ROBOT_FOOTPRINT_WIDTH,
):
    half_length = length / 2.0
    half_width = width / 2.0
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return tuple(
        (
            x + cosine * local_x - sine * local_y,
            y + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        )
    )


def _axes(polygon):
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        edge_x = second[0] - first[0]
        edge_y = second[1] - first[1]
        length = math.hypot(edge_x, edge_y)
        yield (-edge_y / length, edge_x / length)


def _projection(polygon, axis):
    values = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
    return min(values), max(values)


def polygons_intersect(first, second):
    for axis in tuple(_axes(first)) + tuple(_axes(second)):
        first_min, first_max = _projection(first, axis)
        second_min, second_max = _projection(second, axis)
        if first_max < second_min or second_max < first_min:
            return False
    return True


def _point_segment_distance(point, segment_start, segment_end):
    edge_x = segment_end[0] - segment_start[0]
    edge_y = segment_end[1] - segment_start[1]
    squared_length = edge_x * edge_x + edge_y * edge_y
    if squared_length == 0.0:
        return math.hypot(
            point[0] - segment_start[0],
            point[1] - segment_start[1],
        )
    parameter = (
        (point[0] - segment_start[0]) * edge_x
        + (point[1] - segment_start[1]) * edge_y
    ) / squared_length
    parameter = max(0.0, min(1.0, parameter))
    nearest_x = segment_start[0] + parameter * edge_x
    nearest_y = segment_start[1] + parameter * edge_y
    return math.hypot(point[0] - nearest_x, point[1] - nearest_y)


def polygon_clearance(first, second):
    if polygons_intersect(first, second):
        return 0.0
    distances = []
    for polygon, other in ((first, second), (second, first)):
        for point in polygon:
            for edge_start, edge_end in zip(other, other[1:] + other[:1]):
                distances.append(
                    _point_segment_distance(point, edge_start, edge_end)
                )
    return min(distances)


def obstacle_clearances(
    x,
    y,
    yaw,
    obstacles=V1_OBSTACLES,
    length=ROBOT_FOOTPRINT_LENGTH,
    width=ROBOT_FOOTPRINT_WIDTH,
):
    footprint = footprint_corners(x, y, yaw, length, width)
    return {
        obstacle.name: polygon_clearance(footprint, obstacle.corners)
        for obstacle in obstacles
    }
