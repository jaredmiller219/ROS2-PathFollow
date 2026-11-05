import math
from types import SimpleNamespace

import pytest

from occupancy_grid import OccupancyGrid
from obstacle_avoidance import ObstacleAvoidance


def make_scan(ranges, angle_min=0.0, angle_increment=0.0):
    return SimpleNamespace(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.1,
        range_max=10.0,
    )


def test_scan_returns_are_mapped_in_world_coordinates():
    grid = OccupancyGrid()
    scan = make_scan([1.0], angle_min=0.0)

    assert grid.update_from_scan(scan, (2.0, 3.0), math.pi / 2)

    assert grid.has_scan
    assert len(grid.occupied_cells) == 1
    point = next(iter(grid.occupied_cells.values()))
    assert point == pytest.approx((2.0, 4.0))


def test_invalid_ranges_are_ignored():
    grid = OccupancyGrid()
    scan = make_scan([float("nan"), float("inf"), 0.05, 2.0], angle_increment=0.1)

    assert grid.update_from_scan(scan, (0.0, 0.0), 0.0)

    assert len(grid.occupied_cells) == 1
    assert next(iter(grid.occupied_cells.values())) == pytest.approx(
        (2.0 * math.cos(0.3), 2.0 * math.sin(0.3))
    )


def test_nearest_obstacle_ahead_is_found_in_the_path_corridor():
    grid = OccupancyGrid()
    grid.update_from_scan(
        make_scan([1.2, 0.7, 0.9], angle_min=-0.2, angle_increment=0.2),
        (0.0, 0.0),
        0.0,
    )

    obstacle = grid.closest_obstacle_ahead((0.0, 0.0), 0.0, 0.15)

    assert obstacle == pytest.approx((0.7, 0.0))


def test_avoidance_moves_aside_passes_and_returns_to_the_path():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([0.7]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.MOVE_ASIDE
    side_step = avoidance.waypoint
    assert side_step is not None
    assert avoidance.update(side_step) == ObstacleAvoidance.PASS_OBSTACLE

    pass_point = avoidance.waypoint
    assert pass_point is not None
    assert avoidance.update(pass_point) == ObstacleAvoidance.RETURN_TO_PATH

    return_point = avoidance.waypoint
    assert return_point is not None
    assert return_point[1] == pytest.approx(0.0)
    assert avoidance.update(return_point) == ObstacleAvoidance.GO_TO_GOAL


def test_avoidance_chooses_the_side_with_more_clearance():
    grid = OccupancyGrid()
    grid.update_from_scan(
        make_scan([0.7, 0.5], angle_increment=math.pi / 2),
        (0.0, 0.0),
        0.0,
    )
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.MOVE_ASIDE
    assert avoidance.waypoint is not None
    assert avoidance.waypoint[1] < 0.0


def test_avoidance_keeps_driving_when_obstacle_is_not_near():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([1.2]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.GO_TO_GOAL
    assert avoidance.waypoint is None


def test_short_goal_ignores_obstacle_safely_beyond_goal():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([0.7]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (0.2, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.GO_TO_GOAL
    assert avoidance.waypoint is None


def test_short_goal_is_blocked_if_obstacle_is_within_goal_clearance():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([0.5]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (0.2, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.BLOCKED


def test_avoidance_stops_if_both_bypass_routes_are_blocked():
    grid = OccupancyGrid()
    grid.update_from_scan(
        make_scan(
            [0.5, 0.7, 0.5],
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 2,
        ),
        (0.0, 0.0),
        0.0,
    )
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.BLOCKED
