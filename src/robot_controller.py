"""
This file defines the RobotController class, which serves as the main controller for a robot. 
It integrates LiDAR and odometry sensor data to facilitate navigation and obstacle avoidance. 
The RobotController class initializes and manages instances of the LidarSensor and OdometryTracker classes, 
allowing the robot to collect sensor data, process it,
and make navigation decisions based on the information received from these sensors.
"""

# main.py remains the program entry point: it initializes ROS 2, obtains the
# user's goal distance, creates the controller, spins the executor, and
# performs overall shutdown and cleanup.

import math

from geometry_msgs.msg import Twist
from rclpy.node import Node

from lidar import LidarSensor
from odometry import OdometryTracker
from obstacle_avoidance import ObstacleAvoidance

class RobotController(Node):
    OBSTACLE_DISTANCE_THRESHOLD = 0.5
    GOAL_TOLERANCE = 0.1
    CONTROL_PERIOD_SECONDS = 0.1
    MAX_LINEAR_SPEED = 0.15
    MAX_ANGULAR_SPEED = 0.8
    HEADING_GAIN = 1.5

    def __init__(self, goal_distance: float):
        if not isinstance(goal_distance, (int, float)) or not math.isfinite(goal_distance) or goal_distance < 0:
            raise ValueError("goal_distance must be a finite, non-negative number")

        super().__init__("robot_controller")

        self.goal_distance = float(goal_distance)
        self.lidar = LidarSensor()
        self.odometry = OdometryTracker()

        # Set the goal position after the first odometry reading establishes
        # the robot's actual starting pose.
        self.start_position = None
        self.start_orientation = None
        self.goal_position = None

        self.navigation_state = "WAITING_FOR_SENSORS"
        self.obstacle_avoidance = None

        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_timer = self.create_timer(
            self.CONTROL_PERIOD_SECONDS,
            self.control_cycle,
        )

    def sensors_are_ready(self) -> bool:
        """Return whether odometry and a usable front-distance reading exist."""
        if not self.odometry.has_received_odometry:
            return False

        distance = self.lidar.get_latest_distance()
        return (
            distance is not None
            and not math.isnan(distance)
            and distance != -math.inf
        )

    def control_cycle(self) -> None:
        """Run one sensor-driven navigation update."""
        if not self.sensors_are_ready():
            self.publish_stop()
            return

        current_position = self.odometry.get_position()
        current_orientation = self.odometry.get_orientation()

        if self.goal_position is None:
            self.start_position = current_position
            self.start_orientation = current_orientation
            self.goal_position = (
                current_position[0] + self.goal_distance * math.cos(current_orientation),
                current_position[1] + self.goal_distance * math.sin(current_orientation),
            )
            self.obstacle_avoidance = ObstacleAvoidance(
                self.lidar,
                self.odometry,
                self.start_position,
                self.goal_position,
                obstacle_distance_threshold=self.OBSTACLE_DISTANCE_THRESHOLD,
            )
            self.navigation_state = "GO_TO_GOAL"

        if self.has_reached_goal(current_position):
            self.navigation_state = "GOAL_REACHED"
            self.publish_stop()
            return

        lidar_distance = self.lidar.get_latest_distance()
        if self.navigation_state == "GO_TO_GOAL":
            if lidar_distance <= self.OBSTACLE_DISTANCE_THRESHOLD:
                self.handle_obstacle(
                    current_position,
                    current_orientation,
                    lidar_distance,
                )
            else:
                self.navigate_toward_goal(current_position, current_orientation)
        elif self.navigation_state == "WALL_FOLLOW":
            self.handle_obstacle(
                current_position,
                current_orientation,
                lidar_distance,
            )
        elif self.navigation_state == "GOAL_REACHED":
            self.publish_stop()
        else:
            self.get_logger().error(
                f"Unsupported navigation state: {self.navigation_state}"
            )
            self.publish_stop()

    def has_reached_goal(self, current_position: tuple[float, float]) -> bool:
        """Return whether the robot is within tolerance of its target."""
        if self.goal_position is None:
            return False

        remaining_distance = math.hypot(
            self.goal_position[0] - current_position[0],
            self.goal_position[1] - current_position[1],
        )
        return remaining_distance <= self.GOAL_TOLERANCE

    def navigate_toward_goal(
        self,
        current_position: tuple[float, float],
        current_orientation: float,
    ) -> None:
        """Publish a bounded velocity command toward the goal."""
        if self.goal_position is None:
            self.publish_stop()
            return

        goal_bearing = math.atan2(
            self.goal_position[1] - current_position[1],
            self.goal_position[0] - current_position[0],
        )
        heading_error = math.atan2(
            math.sin(goal_bearing - current_orientation),
            math.cos(goal_bearing - current_orientation),
        )

        command = Twist()
        command.angular.z = max(
            -self.MAX_ANGULAR_SPEED,
            min(self.MAX_ANGULAR_SPEED, self.HEADING_GAIN * heading_error),
        )
        if abs(heading_error) < math.pi / 4:
            command.linear.x = self.MAX_LINEAR_SPEED * max(
                0.0,
                math.cos(heading_error),
            )

        self.command_publisher.publish(command)

    def handle_obstacle(
        self,
        current_position: tuple[float, float],
        current_orientation: float,
        lidar_distance: float,
    ) -> None:
        """Update obstacle state and stop when safe wall-follow data is absent."""
        del lidar_distance

        if self.obstacle_avoidance is None:
            self.publish_stop()
            return

        avoidance_state = self.obstacle_avoidance.update_state()
        if avoidance_state is None:
            self.publish_stop()
            return

        self.navigation_state = avoidance_state
        if avoidance_state == "GO_TO_GOAL":
            self.navigate_toward_goal(
                current_position,
                current_orientation,
            )
            return

        # Front-only clearance cannot determine a safe wall-following direction.
        self.publish_stop()

    def publish_stop(self) -> None:
        """Publish a zero-velocity command."""
        self.command_publisher.publish(Twist())

    def shutdown_controller(self) -> None:
        """Stop the robot and release resources owned by this controller."""
        self.publish_stop()
        self.control_timer.cancel()
        self.destroy_timer(self.control_timer)
        self.lidar.destroy_node()
        self.odometry.destroy_node()
        self.destroy_node()