"""Track the robot's pose and total distance traveled from odometry."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdometryTracker(Node):
    def __init__(self):
        super().__init__("odometry_tracker")
        self.position = (0.0, 0.0)
        self.orientation = 0.0
        self.distance_traveled = 0.0
        self.last_position = None
        self.has_received_odometry = False

        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odometry_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    def get_position(self) -> tuple[float, float]:
        return self.position

    def get_orientation(self) -> float:
        return self.orientation

    def get_distance_traveled(self) -> float:
        return self.distance_traveled

    def odometry_callback(self, msg: Odometry) -> None:
        """Update pose and add this odometry step to the distance total."""
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.orientation = self._extract_orientation(msg)
        self.has_received_odometry = True

        if self.last_position is not None:
            self.distance_traveled += math.dist(self.last_position, position)

        self.position = position
        self.last_position = position

    @staticmethod
    def _extract_orientation(msg: Odometry) -> float:
        """Convert the odometry quaternion to a planar heading."""
        orientation = msg.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y**2 + orientation.z**2
        )
        return math.atan2(sin_yaw, cos_yaw)
