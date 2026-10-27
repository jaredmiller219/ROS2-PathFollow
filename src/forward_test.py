"""Run a short, LiDAR-guarded forward-motion test on the TurtleBot."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from lidar import LidarSensor


class TimedForwardDriver(Node):
    """Publish forward velocity for a bounded time while monitoring LiDAR."""

    DURATION_SECONDS = 3.0
    SENSOR_START_TIMEOUT_SECONDS = 10.0
    CONTROL_PERIOD_SECONDS = 0.05
    FORWARD_SPEED_METERS_PER_SECOND = 0.1
    MINIMUM_CLEARANCE_METERS = 0.13

    def __init__(self, lidar: LidarSensor):
        super().__init__("timed_forward_driver")
        self.lidar = lidar
        self.started_at = None
        self.done = False

        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_timer = self.create_timer(
            self.CONTROL_PERIOD_SECONDS,
            self.movement_callback,
        )
        self.sensor_wait_started_at = time.monotonic()

    def movement_callback(self) -> None:
        """Publish forward motion, or stop if time/sensor safety limits are met."""
        if self.done:
            return

        distance = self.lidar.get_latest_distance()
        if distance is None:
            self.publish_stop()
            if (
                time.monotonic() - self.sensor_wait_started_at
                >= self.SENSOR_START_TIMEOUT_SECONDS
            ):
                self.get_logger().error(
                    "No LiDAR scan received before timeout; forward test aborted."
                )
                self.done = True
            return

        if math.isnan(distance) or distance == -math.inf or distance < 0.0:
            self.publish_stop()
            self.get_logger().error(
                "LiDAR returned an invalid distance; forward test aborted."
            )
            self.done = True
            return

        if distance <= self.MINIMUM_CLEARANCE_METERS:
            self.publish_stop()
            self.get_logger().warning(
                "Obstacle is within 0.13 m of the bumper; forward test stopped."
            )
            self.done = True
            return

        now = time.monotonic()
        if self.started_at is None:
            self.started_at = now
            self.get_logger().info(
                "LiDAR is ready; moving forward for up to 3 seconds."
            )

        if now - self.started_at >= self.DURATION_SECONDS:
            self.publish_stop()
            self.get_logger().info("Three-second forward test complete.")
            self.done = True
            return

        command = Twist()
        command.linear.x = self.FORWARD_SPEED_METERS_PER_SECOND
        self.command_publisher.publish(command)

    def publish_stop(self) -> None:
        """Publish a zero-velocity command."""
        self.command_publisher.publish(Twist())

    def shutdown_driver(self) -> None:
        self.publish_stop()
        self.control_timer.cancel()
        self.destroy_timer(self.control_timer)


def main(args=None) -> None:
    driver = None
    lidar = None
    executor = None

    try:
        rclpy.init(args=args)
        lidar = LidarSensor()
        driver = TimedForwardDriver(lidar)

        executor = SingleThreadedExecutor()
        executor.add_node(lidar)
        executor.add_node(driver)

        driver.get_logger().warning(
            "Timed forward test: confirm the area ahead is clear and keep "
            "the robot's stop control accessible."
        )

        while rclpy.ok() and not driver.done:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Forward test interrupted by user.")
    finally:
        if driver is not None:
            driver.shutdown_driver()
        if executor is not None:
            executor.shutdown()
        if driver is not None:
            driver.destroy_node()
        if lidar is not None:
            lidar.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
