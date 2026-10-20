"""
Displays the camera feed from TurtleBot 4.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraFeed(Node):

    def __init__(self):
        super().__init__("camera_feed")

        # Initialize CV Bridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )

        self.get_logger().info("Camera feed node started. Waiting for images...")

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
    node = CameraFeed()
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