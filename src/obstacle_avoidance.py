## obstacle_avoidance.py

"""
Determines how the robot should navigate around detected obstacles.
Uses a simplified Bug algorithm to move around objects and return toward
the original path to continue toward the destination.
"""

import math
from lidar import LidarSensor
from odometry import OdometryTracker


class ObstacleAvoidance:
    """Track Bug-style obstacle avoidance using the controller's sensor nodes."""

    def __init__(
        self,
        lidar: LidarSensor,
        odometry: OdometryTracker,
        start_position: tuple[float, float],
        goal_position: tuple[float, float],
        obstacle_distance_threshold: float = 0.3,
    ):
        if (
            not math.isfinite(obstacle_distance_threshold)
            or obstacle_distance_threshold < 0
        ):
            raise ValueError(
                "obstacle_distance_threshold must be finite and non-negative"
            )

        self.lidar = lidar
        self.odometry = odometry
        self.start_position = start_position
        self.goal_position = goal_position
        self.obstacle_distance_threshold = obstacle_distance_threshold

        x_start, y_start = start_position
        x_goal, y_goal = goal_position

        self.line_a = y_start - y_goal
        self.line_b = x_goal - x_start
        self.line_c = (x_start * y_goal) - (x_goal * y_start)

        self.obstacle_hit_point = None
        self.m_line_tolerance = 0.1
        self.state = "GO_TO_GOAL"


    def distance_to_goal(self, current_position: tuple[float, float]) -> float:
        """Return the Euclidean distance from a position to the goal."""
        x_difference = self.goal_position[0] - current_position[0]
        y_difference = self.goal_position[1] - current_position[1]
        return math.hypot(x_difference, y_difference)

    def check_motion_line(self, current_position: tuple[float, float]) -> bool:
        """
        Return whether a position is within tolerance of the start-goal line.
        """
        x, y = current_position
        numerator = abs((self.line_a * x) + (self.line_b * y) + self.line_c)
        denominator = math.hypot(self.line_a, self.line_b)

        # A zero-length start-to-goal line has no direction or distinct M-line.
        if denominator == 0.0:
            return False

        distance_from_line = numerator / denominator

        # Check if this perpendicular distance is smaller than your threshold:
            # If it is smaller, return True (the robot is on the line)
            # Otherwise, return False (the robot is not on the line)
        return distance_from_line <= self.m_line_tolerance


    def robot_leave(self, current_position: tuple[float, float]) -> bool:
        if self.obstacle_hit_point is None:
            return False

        current_distance = self.distance_to_goal(current_position)
        hit_distance = self.distance_to_goal(self.obstacle_hit_point)
        return (
            self.check_motion_line(current_position)
            and current_distance < hit_distance
        )
    
    def update_state(self) -> str | None:
        """Update avoidance state from current sensors; return None if unready."""
        if not self.odometry.has_received_odometry:
            return None

        lidar_distance = self.lidar.get_latest_distance()
        if (
            lidar_distance is None
            or math.isnan(lidar_distance)
            or lidar_distance == -math.inf
        ):
            return None

        current_position = self.odometry.get_position()

        if self.state == "GO_TO_GOAL":
            if lidar_distance <= self.obstacle_distance_threshold:
                self.obstacle_hit_point = current_position
                self.state = "WALL_FOLLOW"
        elif self.state == "WALL_FOLLOW":
            if self.robot_leave(current_position):
                self.obstacle_hit_point = None
                self.state = "GO_TO_GOAL"
        else:
            raise ValueError(f"Unsupported obstacle-avoidance state: {self.state}")

        return self.state
