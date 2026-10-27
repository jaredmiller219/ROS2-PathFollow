## lidar.py

"""
Reads LiDAR sensor data from the TurtleBot 4 to detect obstacles and
measure their distance from the robot. Provides distance information
that the navigation system uses to determine whether the path is clear.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarSensor(Node):
    def __init__(self):
        """
        Initialize node and subscribe to the LiDAR topic.
        """
        super().__init__('Lidar_Sensor')
        
        # Inspect a narrow forward field of view to detect obstacles in the
        # robot's immediate direction of travel.
        self.FOV_ANGLE = 30.0  # FOV angle in degrees
        self.theta_max = np.radians(self.FOV_ANGLE / 2.0)

        self.SENSOR_TO_BUMPER_DISTANCE = 0.2032  # Offset in meters.
        self.latest_distance = None

        # Connect to TurtleBot 4 LiDAR sensor
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            rclpy.qos.qos_profile_sensor_data
        )

    def extract_scan_data(self, msg: LaserScan):
        """
        Return scan ranges and the angle of each range in radians.
        """
        ranges = np.array(msg.ranges)
        if len(ranges) == 0:
            return ranges, np.array([])

        angle_increment = getattr(msg, "angle_increment", 0.0)
        if angle_increment:
            angles = msg.angle_min + np.arange(len(ranges)) * angle_increment
        else:
            # Keep compatibility with scans/tests that provide only the endpoints.
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        return ranges, angles

    def get_valid_ranges(
        self,
        ranges,
        angles,
        range_min=0.0,
        range_max=math.inf,
    ):
        """
        Keep valid measurements within the forward field of view and range limits.
        """
        ranges = np.asarray(ranges)
        angles = np.asarray(angles)
        if ranges.shape != angles.shape:
            raise ValueError("ranges and angles must have the same shape")

        # Keep only the front view
        fov = (angles >= -self.theta_max) & (angles <= self.theta_max)
        fov_values = ranges[fov]

        # LaserScan specifies that measurements outside these limits are invalid.
        valid_values = (
            np.isfinite(fov_values)
            & (fov_values > 0.0)
            & (fov_values >= range_min)
            & (fov_values <= range_max)
        )
        return fov_values[valid_values]

    def get_latest_distance(self):
        """
        Return the most recently measured distance to a front obstacle.
        """
        return self.latest_distance

    def calculate_final_distance(
        self,
        ranges,
        angles,
        range_min=0.0,
        range_max=math.inf,
    ):
        """
        Return the nearest valid front obstacle distance from the bumper.
        """
        valid_distances = self.get_valid_ranges(
            ranges,
            angles,
            range_min,
            range_max,
        )

        if len(valid_distances) > 0:
            distance_to_closest_object = np.min(valid_distances)
        else:
            distance_to_closest_object = float("inf")

        # This can be done with one line, but just make it more known what the result is.
        distance_with_sensor_offset = distance_to_closest_object - self.SENSOR_TO_BUMPER_DISTANCE
        return distance_with_sensor_offset

    def scan_callback(self, msg: LaserScan):
        """
        Receive a scan, update the latest front-obstacle distance, and return it.
        """
        ranges, angles = self.extract_scan_data(msg)
        final_distance = self.calculate_final_distance(
            ranges,
            angles,
            getattr(msg, "range_min", 0.0),
            getattr(msg, "range_max", math.inf),
        )

        # Store the latest distance for other parts of the system to access
        self.latest_distance = final_distance

        return final_distance