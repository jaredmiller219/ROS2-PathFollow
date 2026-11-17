import math
from types import SimpleNamespace

import pytest

from obstacle_avoidance import ObstacleAvoidance


def make_scan(ranges, angle_min=0.0, angle_increment=0.0):
    return SimpleNamespace(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.1,
        range_max=10.0,
    )


def run_all_legs(avoidance):
    states = []
    while avoidance.current_leg is not None:
        states.append(avoidance.state)
        avoidance.finish_leg()
    states.append(avoidance.state)
    return states


def test_avoidance_turns_drives_and_returns_around_obstacle():
    scan = make_scan([0.8])
    avoidance = ObstacleAvoidance((0.0, 0.0), (4.0, 0.0))

    # Obstacle detected far away - keep driving forward
    assert avoidance.update((0.0, 0.0), 0.0, scan) == ObstacleAvoidance.GO_TO_GOAL
    assert avoidance.current_leg is None

    # When 2 feet away (0.61m), state changes to OBJECT_DETECTED
    assert avoidance.update((0.19, 0.0), 0.0, scan) == ObstacleAvoidance.OBJECT_DETECTED

    # When close to obstacle (0.5m away), start the maneuver
    assert avoidance.update((0.3, 0.0), 0.0, scan) == ObstacleAvoidance.DRIVE_UP
    leg = avoidance.current_leg
    assert leg is not None
    assert leg[0] == ObstacleAvoidance.DRIVE_UP

    states = run_all_legs(avoidance)

    assert states == [
        ObstacleAvoidance.DRIVE_UP,
        ObstacleAvoidance.TURN_ASIDE,
        ObstacleAvoidance.DRIVE_ASIDE,
        ObstacleAvoidance.TURN_FORWARD,
        ObstacleAvoidance.PASS_OBSTACLE,
        ObstacleAvoidance.TURN_TO_PATH,
        ObstacleAvoidance.RETURN_TO_PATH,
        ObstacleAvoidance.TURN_FORWARD,
        ObstacleAvoidance.GO_TO_GOAL,
    ]


def test_avoidance_keeps_driving_when_obstacle_is_not_near():
    scan = make_scan([2.0])
    avoidance = ObstacleAvoidance((0.0, 0.0), (4.0, 0.0))

    assert avoidance.update((0.0, 0.0), 0.0, scan) == ObstacleAvoidance.GO_TO_GOAL
    assert avoidance.current_leg is None


def test_short_goal_ignores_obstacle_beyond_goal_with_clearance():
    scan = make_scan([0.7])
    avoidance = ObstacleAvoidance((0.0, 0.0), (0.2, 0.0))

    assert avoidance.update((0.0, 0.0), 0.0, scan) == ObstacleAvoidance.GO_TO_GOAL
    assert avoidance.current_leg is None


def test_short_goal_stops_when_obstacle_is_too_close_for_bypass():
    scan = make_scan([0.1])
    avoidance = ObstacleAvoidance((0.0, 0.0), (0.2, 0.0))

    # Obstacle detected at 0.1m, within maneuver start distance
    assert avoidance.update((0.0, 0.0), 0.0, scan) == ObstacleAvoidance.DRIVE_UP


def test_avoidance_stops_if_both_bypass_routes_are_blocked():
    scan = make_scan(
        [0.13, 0.1, 0.13],
        angle_min=-1.6,
        angle_increment=1.6,
    )
    avoidance = ObstacleAvoidance((0.0, 0.0), (0.4, 0.0))

    # Obstacle at 0.1m is within maneuver start distance
    assert avoidance.update((0.0, 0.0), 0.0, scan) == ObstacleAvoidance.DRIVE_UP
