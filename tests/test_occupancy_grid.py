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


def run_all_legs(avoidance):
    states = []
    while avoidance.current_leg is not None:
        states.append(avoidance.state)
        avoidance.finish_leg()
    states.append(avoidance.state)
    return states


def test_avoidance_turns_drives_and_returns_around_obstacle():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([0.8]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.TURN_ASIDE
    leg = avoidance.current_leg
    assert leg is not None
    assert leg[0] == ObstacleAvoidance.TURN_ASIDE

    states = run_all_legs(avoidance)

    assert states == [
        ObstacleAvoidance.TURN_ASIDE,
        ObstacleAvoidance.DRIVE_ASIDE,
        ObstacleAvoidance.TURN_FORWARD,
        ObstacleAvoidance.PASS_OBSTACLE,
        ObstacleAvoidance.TURN_TO_PATH,
        ObstacleAvoidance.RETURN_TO_PATH,
        ObstacleAvoidance.TURN_FORWARD,
        ObstacleAvoidance.GO_TO_GOAL,
    ]


def test_avoidance_measures_obstacle_width_for_aside_distance():
    grid = OccupancyGrid()
    grid.update_from_scan(
        make_scan(
            [0.8, 0.8, 0.8],
            angle_min=-0.2,
            angle_increment=0.2,
        ),
        (0.0, 0.0),
        0.0,
    )
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    avoidance.update((0.0, 0.0))
    avoidance.finish_leg()

    leg = avoidance.current_leg
    assert leg is not None
    assert leg[0] == ObstacleAvoidance.DRIVE_ASIDE
    width = 0.8 * 2 * math.sin(0.2)
    expected = width / 2 + avoidance.ROBOT_RADIUS + avoidance.EDGE_MARGIN
    assert leg[2] == pytest.approx(expected, abs=0.05)


def test_blocking_obstacle_reports_lateral_extent():
    grid = OccupancyGrid()
    grid.update_from_scan(
        make_scan(
            [0.8, 0.8, 0.8],
            angle_min=-0.2,
            angle_increment=0.2,
        ),
        (0.0, 0.0),
        0.0,
    )

    extent = grid.blocking_obstacle((0.0, 0.0), 0.0, 0.4, 1.5)

    assert extent is not None
    assert extent.along == pytest.approx(0.8 * math.cos(0.2))
    assert extent.lat_min == pytest.approx(-0.8 * math.sin(0.2))
    assert extent.lat_max == pytest.approx(0.8 * math.sin(0.2))


def test_avoidance_chooses_the_side_with_more_clearance():
    grid = OccupancyGrid()
    grid.has_scan = True
    grid.occupied_cells = {
        (0, 0): (0.8, 0.0),
        (1, 0): (0.8, 0.35),
        (2, 0): (0.15, 0.75),
    }
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.TURN_ASIDE
    leg = avoidance.current_leg
    assert leg is not None
    assert leg[1] == pytest.approx(-math.pi / 2)


def test_avoidance_keeps_driving_when_obstacle_is_not_near():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([2.0]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.GO_TO_GOAL
    assert avoidance.current_leg is None


def test_short_goal_ignores_obstacle_beyond_goal_with_clearance():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([0.7]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (0.2, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.GO_TO_GOAL
    assert avoidance.current_leg is None


def test_short_goal_stops_when_obstacle_is_too_close_for_bypass():
    grid = OccupancyGrid()
    grid.update_from_scan(make_scan([0.1]), (0.0, 0.0), 0.0)
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (0.2, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.BLOCKED


def test_avoidance_stops_if_both_bypass_routes_are_blocked():
    grid = OccupancyGrid()
    grid.update_from_scan(
        make_scan(
            [0.13, 0.1, 0.13],
            angle_min=-1.6,
            angle_increment=1.6,
        ),
        (0.0, 0.0),
        0.0,
    )
    avoidance = ObstacleAvoidance(grid, (0.0, 0.0), (0.4, 0.0))

    assert avoidance.update((0.0, 0.0)) == ObstacleAvoidance.BLOCKED
