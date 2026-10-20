"""
Reads LiDAR sensor data from the TurtleBot 4 to detect obstacles directly in front.
Displays camera feed with LiDAR distance overlay.
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

        # Store latest distance for overlay
        self.latest_distance = float("inf")

        # Subscribe to TurtleBot 4 LiDAR topic using sensor QoS
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        # Try common TurtleBot 4 camera topics
        camera_topics = [
            "/camera/image_raw",
            "/oakd/color/image_raw",
            "/camera/color/image_raw",
            "/camera/rgb/image_raw",
        ]

        self.camera_subscribed = False
        for topic in camera_topics:
            try:
                self.camera_subscription = self.create_subscription(
                    Image,
                    topic,
                    self.image_callback,
                    10,
                )
                self.get_logger().info(f"Subscribed to camera topic: {topic}")
                self.camera_subscribed = True
                break
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to {topic}: {e}")
                continue

        if not self.camera_subscribed:
            self.get_logger().error("Could not subscribe to any camera topic")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_samples = len(ranges)

        if num_samples == 0:
            self.latest_distance = float("inf")
            return

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

        self.latest_distance = final_distance
        self.get_logger().info(
            f"Distance to front obstacle: {final_distance:.2f} meters"
        )

    def image_callback(self, msg: Image):
        try:
            self.get_logger().info("Received camera image")
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Overlay distance text on the image
            if self.latest_distance != float("inf"):
                distance_text = f"Distance: {self.latest_distance:.2f}m"
                color = (0, 255, 0) if self.latest_distance > 0.5 else (0, 0, 255)
            else:
                distance_text = "Distance: No obstacle detected"
                color = (255, 255, 255)

            # Add text with background for better visibility
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            thickness = 2
            (text_width, text_height), baseline = cv2.getTextSize(
                distance_text, font, font_scale, thickness
            )

            # Draw semi-transparent background rectangle
            overlay = cv_image.copy()
            cv2.rectangle(
                overlay,
                (10, 10),
                (10 + text_width + 20, 10 + text_height + 20),
                (0, 0, 0),
                -1,
            )
            cv2.addWeighted(overlay, 0.5, cv_image, 0.5, 0, cv_image)

            # Draw text
            cv2.putText(
                cv_image,
                distance_text,
                (20, 35),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA,
            )

            # Display the image
            cv2.imshow("TurtleBot 4 Camera Feed with LiDAR", cv_image)
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