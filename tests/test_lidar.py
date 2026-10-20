## test_lidar.py

import math
import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
import rclpy

# Import your node
from lidar import LidarSensor


@pytest.fixture(scope="module", autouse=True)
def rclpy_init():
    """Starts rclpy context before tests run and shuts down after."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    """Provides a fresh LidarSensor node instance for each test."""
    sensor = LidarSensor()
    yield sensor
    sensor.destroy_node()


def make_scan_msg(ranges_list, angle_min=-math.pi, angle_max=math.pi):
    """Helper function to mock ROS LaserScan messages."""
    msg = LaserScan()
    msg.angle_min = float(angle_min)
    msg.angle_max = float(angle_max)
    msg.ranges = [float(r) for r in ranges_list]
    return msg


# --- TESTS ---

def test_extract_scan_data(node):
    """Checks range & angle extraction from raw message."""
    msg = make_scan_msg([1.0, 2.0, 3.0], angle_min=-1.0, angle_max=1.0)
    ranges, angles = node.extract_scan_data(msg)

    assert len(ranges) == 3
    assert len(angles) == 3
    np.testing.assert_almost_equal(angles, np.array([-1.0, 0.0, 1.0]))


def test_get_valid_ranges_fov_filtering(node):
    """Verifies that values outside the 60-degree FOV are dropped."""
    # FOV = 60° (range: -30° to +30°)
    angles = np.radians(np.array([-45, -20, 0, 20, 45]))
    ranges = np.array([1.0, 2.0, 3.0, 4.0, 5.0])

    valid = node.get_valid_ranges(ranges, angles)

    # Only indices 1, 2, 3 (-20°, 0°, 20°) should survive
    np.testing.assert_array_equal(valid, np.array([2.0, 3.0, 4.0]))


def test_get_valid_ranges_error_filtering(node):
    """Verifies that inf, NaN, 0, and negative values are dropped."""
    angles = np.radians(np.array([-10, -5, 0, 5, 10]))
    ranges = np.array([1.5, float("inf"), float("nan"), 0.0, -0.5])

    valid = node.get_valid_ranges(ranges, angles)

    assert len(valid) == 1
    assert valid[0] == 1.5


def test_calculate_final_distance(node):
    """Verifies closest distance calculation with bumper offset applied."""
    angles = np.radians(np.array([-10, 0, 10]))
    ranges = np.array([2.0, 0.8, 1.5])  # Min value is 0.8m

    # 0.8m - 0.1m (bumper offset) = 0.7m
    distance = node.calculate_final_distance(ranges, angles)
    assert pytest.approx(distance, abs=1e-4) == 0.7


def test_calculate_final_distance_no_obstacles(node):
    """Verifies infinity returned when no obstacles are within FOV."""
    angles = np.radians(np.array([-60, 60]))  # Both outside 60° FOV
    ranges = np.array([1.0, 1.0])

    distance = node.calculate_final_distance(ranges, angles)
    assert math.isinf(distance)