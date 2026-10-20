"""
Reads LiDAR sensor data from the TurtleBot 4 to detect obstacles directly in front.
Also displays the camera feed.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2


class LidarSensor(Node):

    def __init__(self):
        super().__init__("Lidar_Sensor")

        # Constants
        self.FOV_ANGLE = 90.0  # Cone of view in degrees (e.g., +/- 15 deg in front)
        self.SENSOR_TO_BUMPER_DISTANCE = 0.2032  # Offset in meters

        # Initialize CV Bridge for camera
        self.bridge = CvBridge()

        # Subscribe to TurtleBot 4 LiDAR topic using sensor QoS
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        # Subscribe to camera image topic
        self.camera_subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_samples = len(ranges)

        if num_samples == 0:
            return float("inf")

        # Determine how many array indices correspond to half of our FOV
        fov_rad = np.radians(self.FOV_ANGLE / 2.0)
        idx_window = int(fov_rad / msg.angle_increment)

        # Index 0 is straight ahead. Grab the front-left and front-right slices
        front_right = ranges[:idx_window]
        front_left = ranges[-idx_window:]
        front_cone = np.concatenate((front_left, front_right))

        # Filter out invalid ranges (0.0, inf, nan, or out-of-bounds)
        valid_ranges = front_cone[
            np.isfinite(front_cone)
            & (front_cone > msg.range_min)
            & (front_cone < msg.range_max)
        ]

        if len(valid_ranges) > 0:
            closest_raw = np.min(valid_ranges)
            final_distance = closest_raw - self.SENSOR_TO_BUMPER_DISTANCE
        else:
            final_distance = float("inf")

        self.get_logger().info(
            f"Distance to front obstacle: {final_distance:.2f} meters"
        )
        return final_distance

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Display the image
            cv2.imshow("TurtleBot 4 Camera Feed", cv_image)

            # Update the display (required for cv2.imshow to work)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()