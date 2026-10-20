## lidar.py

"""
Reads LiDAR sensor data from the TurtleBot 4 to detect obstacles and
measure their distance from the robot. Provides distance information
that the navigation system uses to determine whether the path is clear.
"""

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
        
        # Constants
        self.FOV_ANGLE = 90.0  # FOV angle in degrees
        self.SENSOR_TO_BUMPER_DISTANCE = 0.2032  # Offset in meters. Actually go and measure this on the robot.
            
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
        theta_max = np.radians(self.FOV_ANGLE / 2.0)
        
        # Keep only the front view
        fov_mask = (angles >= -theta_max) & (angles <= theta_max)
        d_fov = ranges[fov_mask]
        
        # Remove invalid numbers (like inf, NaN, or 0)
        valid_mask = np.isfinite(d_fov) & (d_fov > 0.0)
        return d_fov[valid_mask]

    def calculate_final_distance(self, ranges, angles):
        """
        Takes raw data, cleans it, finds the closest object, and subtracts the bumper offset.
        """
        d_valid = self.get_valid_ranges(ranges, angles)
        
        distance_to_closest_object = np.min(d_valid) if len(d_valid) > 0 else float("inf")
        distance_with_sensor_offset = distance_to_closest_object - self.SENSOR_TO_BUMPER_DISTANCE
        
        return distance_with_sensor_offset

    def scan_callback(self, msg: LaserScan):
        """
        Main trigger: Receives the scan, calculates the final distance, and logs it.
        """
        ranges, angles = self.extract_scan_data(msg)
        final_distance = self.calculate_final_distance(ranges, angles)
        
        self.get_logger().info(f"Distance to front obstacle: {final_distance:.2f} meters")
        return final_distance


def main(args=None):
    rclpy.init(args=args)
    node = LidarSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()