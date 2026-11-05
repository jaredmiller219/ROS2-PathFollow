import math

import pytest

from nav_msgs.msg import Odometry
from odometry import OdometryTracker


def make_odometry(x=0.0, y=0.0, yaw=0.0):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def test_tracker_starts_without_odometry():
    tracker = OdometryTracker()

    assert tracker.has_received_odometry is False
    assert tracker.get_position() == (0.0, 0.0)
    assert tracker.get_orientation() == 0.0
    assert tracker.get_distance_traveled() == 0.0


def test_first_odometry_message_sets_pose_without_adding_distance():
    tracker = OdometryTracker()

    tracker.odometry_callback(make_odometry(1.0, 2.0, math.pi / 2))

    assert tracker.has_received_odometry is True
    assert tracker.get_position() == (1.0, 2.0)
    assert tracker.get_orientation() == pytest.approx(math.pi / 2)
    assert tracker.get_distance_traveled() == 0.0


def test_tracker_accumulates_distance_between_updates():
    tracker = OdometryTracker()
    tracker.odometry_callback(make_odometry(0.0, 0.0))

    tracker.odometry_callback(make_odometry(3.0, 4.0, math.pi))
    tracker.odometry_callback(make_odometry(3.0, 6.0, -math.pi / 2))

    assert tracker.get_position() == (3.0, 6.0)
    assert tracker.get_orientation() == pytest.approx(-math.pi / 2)
    assert tracker.get_distance_traveled() == pytest.approx(7.0)


def test_orientation_extraction_handles_wrapped_yaw():
    message = make_odometry(yaw=-math.pi + 0.01)

    assert OdometryTracker._extract_orientation(message) == pytest.approx(
        -math.pi + 0.01
    )
