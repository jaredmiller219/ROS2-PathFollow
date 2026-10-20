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
        Grab raw laser numbers.
        """
        ranges = np.array(msg.ranges)
        num_samples = len(ranges)
        return ranges, num_samples

    def get_front_cone(self, ranges, msg):
        """
        Keep only what's right in front of the robot using index-based filtering.
        Index 0 is straight ahead. Grab the front-left and front-right slices.
        """
        fov_rad = np.radians(self.FOV_ANGLE / 2.0)
        idx_window = int(fov_rad / msg.angle_increment)

        front_right = ranges[:idx_window]
        front_left = ranges[-idx_window:]
        front_cone = np.concatenate((front_left, front_right))
        return front_cone

    def get_valid_ranges(self, front_cone, msg):
        """
        Remove invalid numbers (like inf, NaN, or out-of-bounds).
        """
        valid_ranges = front_cone[
            np.isfinite(front_cone)
            & (front_cone > msg.range_min)
            & (front_cone < msg.range_max)
        ]
        return valid_ranges

    def calculate_final_distance(self, valid_ranges):
        """
        Find the closest object and subtract the bumper offset.
        """
        if len(valid_ranges) > 0:
            closest_raw = np.min(valid_ranges)
            final_distance = closest_raw - self.SENSOR_TO_BUMPER_DISTANCE
        else:
            final_distance = float("inf")
        return final_distance

    def scan_callback(self, msg: LaserScan):
        """
        Main trigger: Receives the scan, calculates the final distance, and logs it.
        """
        ranges, num_samples = self.extract_scan_data(msg)

        if num_samples == 0:
            final_distance = float("inf")
        else:
            front_cone = self.get_front_cone(ranges, msg)
            valid_ranges = self.get_valid_ranges(front_cone, msg)
            final_distance = self.calculate_final_distance(valid_ranges)

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