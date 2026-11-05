"""Coordinate sensor data, obstacle avoidance, and robot movement."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from occupancy_grid import OccupancyGrid
from obstacle_avoidance import ObstacleAvoidance
from odometry import OdometryTracker

"""
Each /scan message contains distances at different angles. 
The controller passes that scan to the occupancy grid, 
which turns valid readings into obstacle positions
using the robot's pose (scan_callback, update_from_scan). 
Obstacle avoidance then finds the nearest mapped point 
ahead within the planned path corridor (closest_obstacle_ahead) 
and starts a bypass when it is within the 0.85 m trigger distance 
(obstacle check).
"""

class RobotController(Node):
    # Acceptable distance from the final goal before the robot declares success.
    GOAL_TOLERANCE = 0.03

    # Distance threshold used to decide whether the current waypoint has been reached.
    WAYPOINT_TOLERANCE = 0.10

    # How often the movement controller updates the robot's motion.
    CONTROL_PERIOD_SECONDS = 0.1

    # Maximum forward speed allowed while navigating.
    MAX_LINEAR_SPEED = 0.12

    # Minimum forward speed to keep the robot moving when not turning in place.
    MIN_LINEAR_SPEED = 0.03

    # Maximum angular velocity allowed during turning adjustments.
    MAX_ANGULAR_SPEED = 0.7

    # Angle threshold beyond which the robot rotates in place instead of driving forward.
    TURN_IN_PLACE_THRESHOLD = 0.25

    # Gain applied when computing turn commands toward a target heading.
    TURN_GAIN = 1.5

    @classmethod
    def run(cls, goal_distance: float, args=None) -> None:
        """Initialize ROS, run navigation, and clean up."""
        rclpy.init(args=args)
        controller = None
        executor = None

        try:
            controller = cls(goal_distance)
            executor = SingleThreadedExecutor()
            for node in (controller.odometry, controller):
                executor.add_node(node)

            controller.start()
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            print("Navigation interrupted by user.")
        finally:
            if controller is not None:
                controller.shutdown_controller()
            if executor is not None:
                executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()

    def __init__(self, goal_distance: float):
        if (
            not isinstance(goal_distance, (int, float))
            or not math.isfinite(goal_distance)
            or goal_distance < 0.0
        ):
            raise ValueError("goal_distance must be finite and non-negative")

        super().__init__("robot_controller")
        self.goal_distance = float(goal_distance)

        self.odometry = OdometryTracker()
        self.occupancy_grid = OccupancyGrid()

        self.obstacle_avoidance: ObstacleAvoidance | None = None
        self.start_position: tuple[float, float] | None = None
        self.start_orientation: float | None = None
        self.goal_position: tuple[float, float] | None = None
        self.navigation_state = "WAITING_FOR_SENSORS"

        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.control_timer = None

    def start(self) -> None:
        """Start periodic movement control after executor setup."""
        if self.control_timer is None:
            self.control_timer = self.create_timer(
                self.CONTROL_PERIOD_SECONDS,
                self.movement_callback,
            )

    def scan_callback(self, scan: LaserScan) -> None:
        """Project scan returns into the odometry-based occupancy grid."""
        if not self.odometry.has_received_odometry:
            return

        updated = self.occupancy_grid.update_from_scan(
            scan,
            self.odometry.get_position(),
            self.odometry.get_orientation(),
        )
        if not updated:
            self.get_logger().warning("Ignored an invalid LiDAR scan.")

    def sensors_are_ready(self) -> bool:
        """Wait until odometry and a valid LiDAR scan are available."""
        return self.odometry.has_received_odometry and self.occupancy_grid.has_scan

    def movement_callback(self) -> None:
        """Update the route and publish one movement command."""
        if self.navigation_state in ("GOAL_REACHED", "BLOCKED", "STOPPED"):
            self.publish_stop()
            return

        if not self.sensors_are_ready():
            self.publish_stop()
            return

        current_position = self.odometry.get_position()
        current_orientation = self.odometry.get_orientation()
        self._initialize_route(current_position, current_orientation)

        if self.has_reached_goal(current_position):
            self.navigation_state = "GOAL_REACHED"
            self.publish_stop()
            self.get_logger().info("Destination reached.")
            return

        avoidance = self.obstacle_avoidance
        if avoidance is None:
            self.publish_stop()
            return

        self.navigation_state = avoidance.update(current_position)
        if self.navigation_state == ObstacleAvoidance.BLOCKED:
            self.publish_stop()
            self.get_logger().error(
                "No clear bypass is visible in the occupancy grid; robot stopped."
            )
            return

        target = self._current_target()
        if target is None:
            self.publish_stop()
            return

        if self.navigation_state != ObstacleAvoidance.GO_TO_GOAL:
            clearance = self.occupancy_grid.segment_clearance(
                current_position,
                target,
            )
            if clearance < ObstacleAvoidance.ROBOT_CLEARANCE:
                self.navigation_state = ObstacleAvoidance.BLOCKED
                self.publish_stop()
                self.get_logger().error(
                    "The planned bypass is blocked by a mapped obstacle."
                )
                return

        self.navigate_to_target(
            current_position,
            current_orientation,
            target,
        )

    def _initialize_route(
        self,
        current_position: tuple[float, float],
        current_orientation: float,
    ) -> None:
        if self.goal_position is not None:
            return

        goal_position = (
            current_position[0] + self.goal_distance * math.cos(current_orientation),
            current_position[1] + self.goal_distance * math.sin(current_orientation),
        )
        self.start_position = current_position
        self.start_orientation = current_orientation
        self.goal_position = goal_position
        self.obstacle_avoidance = ObstacleAvoidance(
            self.occupancy_grid,
            current_position,
            goal_position,
        )
        self.navigation_state = ObstacleAvoidance.GO_TO_GOAL

    def _current_target(self) -> tuple[float, float] | None:
        if self.navigation_state == ObstacleAvoidance.GO_TO_GOAL:
            return self.goal_position
        if self.obstacle_avoidance is None:
            return None
        return self.obstacle_avoidance.waypoint

    def has_reached_goal(self, current_position: tuple[float, float]) -> bool:
        """Check whether the robot is physically within goal tolerance."""
        if self.goal_position is None:
            return False

        return math.dist(current_position, self.goal_position) <= self.GOAL_TOLERANCE

    def navigate_to_target(
        self,
        current_position: tuple[float, float],
        current_orientation: float,
        target: tuple[float, float],
    ) -> None:
        """Turn toward a waypoint, then drive toward it."""
        dx = target[0] - current_position[0]
        dy = target[1] - current_position[1]
        distance = math.hypot(dx, dy)
        target_tolerance = (
            self.GOAL_TOLERANCE
            if self.navigation_state == ObstacleAvoidance.GO_TO_GOAL
            else self.WAYPOINT_TOLERANCE
        )
        if distance <= target_tolerance:
            self.publish_stop()
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(desired_heading - current_orientation),
            math.cos(desired_heading - current_orientation),
        )
        command = Twist()

        if abs(heading_error) > self.TURN_IN_PLACE_THRESHOLD:
            command.angular.z = self._clamp(
                self.TURN_GAIN * heading_error,
                -self.MAX_ANGULAR_SPEED,
                self.MAX_ANGULAR_SPEED,
            )
        else:
            command.linear.x = min(
                self.MAX_LINEAR_SPEED,
                max(self.MIN_LINEAR_SPEED, distance * 0.4),
            )
            command.angular.z = self._clamp(
                self.TURN_GAIN * heading_error,
                -self.MAX_ANGULAR_SPEED,
                self.MAX_ANGULAR_SPEED,
            )

        self.command_publisher.publish(command)

    def publish_stop(self) -> None:
        """Publish a zero-velocity command."""
        self.command_publisher.publish(Twist())

    def shutdown_controller(self) -> None:
        """Stop the robot and destroy all controller-owned nodes."""
        if rclpy.ok():
            self.publish_stop()

        if self.control_timer is not None:
            self.control_timer.cancel()
            self.destroy_timer(self.control_timer)

        for node in (self.odometry, self):
            node.destroy_node()

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
