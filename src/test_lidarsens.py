"""
Live Camera Feed for TurtleBot 4 with LiDAR Obstacle Distance Overlay.
Requires opencv-python and cv_bridge.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


class LidarVideoFeed(Node):

    def __init__(self):
        super().__init__("lidar_video_feed")

        # Constants
        self.FOV_ANGLE = 30.0  # Cone of view in degrees
        self.SENSOR_TO_BUMPER = 0.2032  # Bumper offset in meters
        self.latest_distance = float("inf")

        self.bridge = CvBridge()

        # Subscribe to LiDAR topic
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

        # Subscribe to TurtleBot 4 OAK-D RGB Camera feed
        self.image_sub = self.create_subscription(
            Image,
            "/oakd/rgb/preview/image_raw",
            self.image_callback,
            qos_profile_sensor_data,
        )

    def scan_callback(self, msg: LaserScan):
        """Processes LiDAR ranges to calculate distance to front obstacle."""
        ranges = np.array(msg.ranges)
        num_samples = len(ranges)

        if num_samples == 0:
            return

        fov_rad = np.radians(self.FOV_ANGLE / 2.0)
        idx_window = int(fov_rad / msg.angle_increment)

        front_right = ranges[:idx_window]
        front_left = ranges[-idx_window:]
        front_cone = np.concatenate((front_left, front_right))

        valid_ranges = front_cone[
            np.isfinite(front_cone)
            & (front_cone > msg.range_min)
            & (front_cone < msg.range_max)
        ]

        if len(valid_ranges) > 0:
            closest_raw = np.min(valid_ranges)
            self.latest_distance = max(0.0, closest_raw - self.SENSOR_TO_BUMPER)
        else:
            self.latest_distance = float("inf")

    def image_callback(self, msg: Image):
        """Receives camera frame, overlays distance reading, and displays GUI window."""
        try:
            # Convert ROS Image message to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        h, w, _ = cv_image.shape

        # Format display text based on distance
        if np.isinf(self.latest_distance):
            text = "Front Path: CLEAR"
            color = (0, 255, 0)  # Green
        else:
            text = f"Front Distance: {self.latest_distance:.2f} m"
            # Turn text red if object is closer than 0.5m
            color = (0, 0, 255) if self.latest_distance < 0.5 else (0, 255, 255)

        # Draw target box in the center of camera view
        box_width, box_height = 120, 80
        top_left = (int(w / 2 - box_width / 2), int(h / 2 - box_height / 2))
        bottom_right = (int(w / 2 + box_width / 2), int(h / 2 + box_height / 2))
        cv2.rectangle(cv_image, top_left, bottom_right, color, 2)

        # Draw background banner for text readability
        cv2.rectangle(cv_image, (0, 0), (w, 40), (0, 0, 0), -1)

        # Overlay text
        cv2.putText(
            cv_image,
            text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )

        # Display window
        cv2.imshow("TurtleBot 4 Live Feed", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LidarVideoFeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()