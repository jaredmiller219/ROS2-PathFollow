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
        
        # subject to change based on accuracy of objects in front
        # for now, we will just used 90 degrees
        self.FOV_ANGLE = 90.0  # FOV angle in degrees
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
        Grab raw laser numbers and figure out their angles.
        """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        return ranges, angles

    def get_valid_ranges(self, ranges, angles):
        """
        Keep only what's right in front of the robot and throw away errors/bad numbers.
        """
        # Keep only the front view
        fov = (angles >= -self.theta_max) & (angles <= self.theta_max)
        fov_values = ranges[fov]

        # Remove invalid numbers (like inf, NaN, or 0)
        valid_values = np.isfinite(fov_values) & (fov_values > 0.0)
        return fov_values[valid_values]

    def get_latest_distance(self):
        """
        Return the most recently measured distance to a front obstacle.
        """
        return self.latest_distance

    def calculate_final_distance(self, ranges, angles):
        """
        Takes raw data, cleans it, finds the closest object, and subtracts the bumper offset.
        """
        valid_distances = self.get_valid_ranges(ranges, angles)

        if len(valid_distances) > 0:
            distance_to_closest_object = np.min(valid_distances)
        else:
            distance_to_closest_object = float("inf")

        # This can be done with one line, but just make it more known what the result is.
        distance_with_sensor_offset = distance_to_closest_object - self.SENSOR_TO_BUMPER_DISTANCE
        return distance_with_sensor_offset

    def scan_callback(self, msg: LaserScan):
        """
        Main trigger: Receives the scan, calculates the final distance, and logs it.
        """
        ranges, angles = self.extract_scan_data(msg)
        final_distance = self.calculate_final_distance(ranges, angles)

        # Store the latest distance for other parts of the system to access
        self.latest_distance = final_distance

        print(f"Distance to obstacle: {final_distance:.2f} meters", flush=True)

        return final_distance


def main(args=None):
    """Run the LiDAR node and report the latest front-obstacle distance."""
    rclpy.init(args=args)

    try:
        lidar = LidarSensor()
        try:
            rclpy.spin(lidar)
        except KeyboardInterrupt:
            pass
        finally:
            lidar.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()