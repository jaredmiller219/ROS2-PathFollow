import math
from types import SimpleNamespace

import pytest

from geometry_msgs.msg import Twist
from obstacle_avoidance import ObstacleAvoidance
from robot_controller import RobotController


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeOdometry:
    def __init__(self, position=(0.0, 0.0), orientation=0.0, ready=True):
        self.position = position
        self.orientation = orientation
        self.has_received_odometry = ready

    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.orientation


def make_controller(goal_distance=1.0):
    controller = RobotController(goal_distance)
    controller.command_publisher = RecordingPublisher()
    controller.odometry.destroy_node()
    controller.odometry = FakeOdometry()
    return controller


@pytest.mark.parametrize("distance", [-1.0, float("nan"), float("inf"), "1"])
def test_goal_distance_must_be_finite_non_negative_number(distance):
    with pytest.raises(ValueError):
        RobotController(distance)


def test_start_creates_only_one_control_timer():
    controller = make_controller()

    controller.start()
    first_timer = controller.control_timer
    controller.start()

    assert controller.control_timer is first_timer
    assert first_timer.period == controller.CONTROL_PERIOD_SECONDS


def test_scan_is_ignored_until_odometry_is_available():
    controller = make_controller()
    controller.odometry.has_received_odometry = False
    scan = SimpleNamespace()

    controller.scan_callback(scan)

    assert controller.occupancy_grid.has_scan is False


def test_scan_updates_occupancy_grid_after_odometry_is_available():
    controller = make_controller()
    scan = SimpleNamespace(
        ranges=[1.0],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
    )

    controller.scan_callback(scan)

    assert controller.occupancy_grid.has_scan is True
    assert len(controller.occupancy_grid.occupied_cells) == 1


def test_movement_waits_for_sensors_and_publishes_stop():
    controller = make_controller()
    controller.odometry.has_received_odometry = False

    controller.movement_callback()

    assert len(controller.command_publisher.messages) == 1
    command = controller.command_publisher.messages[-1]
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_movement_publishes_forward_command_when_path_is_clear():
    controller = make_controller(goal_distance=1.0)
    controller.occupancy_grid.has_scan = True

    controller.movement_callback()

    command = controller.command_publisher.messages[-1]
    assert controller.navigation_state == ObstacleAvoidance.GO_TO_GOAL
    assert command.linear.x == pytest.approx(controller.MIN_LINEAR_SPEED)
    assert command.angular.z == pytest.approx(0.0)


def test_movement_turns_in_place_when_heading_error_is_large():
    controller = make_controller(goal_distance=1.0)
    controller.occupancy_grid.has_scan = True
    controller.odometry = FakeOdometry()
    controller._initialize_route((0.0, 0.0), 0.0)
    controller.odometry.orientation = math.pi / 2

    controller.movement_callback()

    command = controller.command_publisher.messages[-1]
    assert command.linear.x == 0.0
    assert abs(command.angular.z) == pytest.approx(controller.MAX_ANGULAR_SPEED)


def test_movement_stops_after_goal_is_reached():
    controller = make_controller(goal_distance=1.0)
    controller.occupancy_grid.has_scan = True
    controller.goal_position = (1.0, 0.0)
    controller.odometry = FakeOdometry(position=(1.0, 0.0))

    controller.movement_callback()

    command = controller.command_publisher.messages[-1]
    assert controller.navigation_state == "GOAL_REACHED"
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_movement_stops_when_obstacle_avoidance_is_blocked():
    controller = make_controller()
    controller.occupancy_grid.has_scan = True
    controller.goal_position = (1.0, 0.0)
    controller.obstacle_avoidance = SimpleNamespace(
        update=lambda position: ObstacleAvoidance.BLOCKED
    )

    controller.movement_callback()

    command = controller.command_publisher.messages[-1]
    assert controller.navigation_state == ObstacleAvoidance.BLOCKED
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_publish_stop_publishes_zero_twist():
    controller = make_controller()

    controller.publish_stop()

    command = controller.command_publisher.messages[-1]
    assert isinstance(command, Twist)
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0
