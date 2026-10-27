"""
This file defines the RobotController class, which serves as the main controller for a robot. 
It integrates LiDAR and odometry sensor data to facilitate navigation and obstacle avoidance. 
The RobotController class initializes and manages instances of the LidarSensor and OdometryTracker classes, 
allowing the robot to collect sensor data, process it,
and make navigation decisions based on the information received from these sensors.
"""

# main.py remains the program entry point: it initializes ROS 2, obtains the
# user's goal distance, creates the controller, spins the executor, and
# performs overall shutdown and cleanup.

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from lidar import LidarSensor
from odometry import OdometryTracker
from obstacle_avoidance import ObstacleAvoidance

class RobotController(Node):
    # lidar.py reports distance from the bumper after applying its sensor offset.
    OBSTACLE_DISTANCE_THRESHOLD = 0.3
    GOAL_TOLERANCE = 0.03
    CONTROL_PERIOD_SECONDS = 0.1
    # TurtleBot 4 LiDAR can take a while to spin up, especially over Wi-Fi.
    SENSOR_START_TIMEOUT_SECONDS = 30.0
    # Conservative test speed; the TurtleBot 4 base tops out around 0.3 m/s.
    MAX_LINEAR_SPEED = 0.2
    # Smallest forward command that still reliably moves the base.
    MIN_LINEAR_SPEED = 0.03
    APPROACH_SPEED_GAIN = 0.5

    @classmethod
    def run(cls, goal_distance: float, args=None) -> None:
        """Initialize ROS, run the controller and sensors, then clean up."""
        executor = None
        robot_controller = None
        ros_initialized = False

        try:
            rclpy.init(args=args)
            ros_initialized = True

            robot_controller = cls(goal_distance)
            executor = SingleThreadedExecutor()
            for node in (
                robot_controller.lidar,
                robot_controller.odometry,
                robot_controller,
            ):
                executor.add_node(node)

            robot_controller.start()
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            print("Navigation interrupted by user.")
        finally:
            if robot_controller is not None:
                robot_controller.shutdown_controller()
            if executor is not None:
                executor.shutdown()
            if ros_initialized and rclpy.ok():
                rclpy.shutdown()

    def __init__(self, goal_distance: float):
        if not isinstance(goal_distance, (int, float)) or not math.isfinite(goal_distance) or goal_distance < 0:
            raise ValueError("goal_distance must be a finite, non-negative number")

        super().__init__("robot_controller")

        self.goal_distance = float(goal_distance)
        self.lidar = LidarSensor()
        self.odometry = OdometryTracker()

        # Set the goal position after the first odometry reading establishes
        # the robot's actual starting pose.
        self.start_position = None
        self.start_orientation = None
        self.goal_position = None

        self.navigation_state = "WAITING_FOR_SENSORS"
        self.obstacle_avoidance = None
        self.obstacle_threshold_reported = False

        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_timer = None

    def start(self) -> None:
        """Start periodic movement control after main adds nodes to its executor."""
        if self.control_timer is not None:
            return

        self.control_timer = self.create_timer(
            self.CONTROL_PERIOD_SECONDS,
            self.movement_callback,
        )

    def sensors_are_ready(self) -> bool:
        """Return whether odometry and a usable front-distance reading exist."""
        if not self.odometry.has_received_odometry:
            return False

        distance = self.lidar.get_latest_distance()
        return (
            distance is not None
            and not math.isnan(distance)
            and distance != -math.inf
        )

    def movement_callback(self) -> None:
        """Drive toward the entered odometry goal while monitoring LiDAR."""
        if self.navigation_state in ("GOAL_REACHED", "STOPPED"):
            self.publish_stop()
            return

        if not self.sensors_are_ready():
            self.publish_stop()
            return

        current_position = self.odometry.get_position()
        current_orientation = self.odometry.get_orientation()

        if self.goal_position is None:
            self.start_position = current_position
            self.start_orientation = current_orientation
            self.goal_position = (
                current_position[0] + self.goal_distance * math.cos(current_orientation),
                current_position[1] + self.goal_distance * math.sin(current_orientation),
            )
            self.obstacle_avoidance = ObstacleAvoidance(
                self.lidar,
                self.odometry,
                self.start_position,
                self.goal_position,
                obstacle_distance_threshold=self.OBSTACLE_DISTANCE_THRESHOLD,
            )
            self.navigation_state = "GO_TO_GOAL"

        if self.has_reached_goal(current_position):
            self.navigation_state = "GOAL_REACHED"
            self.publish_stop()
            return

        lidar_distance = self.lidar.get_latest_distance()
        if (
            lidar_distance <= self.OBSTACLE_DISTANCE_THRESHOLD
            and not self.obstacle_threshold_reported
        ):
            print(
                f"Obstacle is {lidar_distance:.2f} m away "
                f"(threshold: {self.OBSTACLE_DISTANCE_THRESHOLD:.2f} m)."
            )
            self.obstacle_threshold_reported = True

        if self.navigation_state == "GO_TO_GOAL":
            if lidar_distance <= self.OBSTACLE_DISTANCE_THRESHOLD:
                self.handle_obstacle()
            else:
                self.navigate_toward_goal(lidar_distance)
        elif self.navigation_state == "WALL_FOLLOW":
            self.handle_obstacle()
        elif self.navigation_state == "GOAL_REACHED":
            self.publish_stop()
        else:
            self.publish_stop()

    def has_reached_goal(self, current_position: tuple[float, float]) -> bool:
        """Return whether the robot has travelled the goal distance.

        Progress is measured along the starting heading, so sideways drift
        from driving straight cannot make the robot miss the goal and keep
        going past it.
        """
        if self.start_position is None or self.start_orientation is None:
            return False

        dx = current_position[0] - self.start_position[0]
        dy = current_position[1] - self.start_position[1]
        progress = (
            dx * math.cos(self.start_orientation)
            + dy * math.sin(self.start_orientation)
        )
        return progress >= self.goal_distance - self.GOAL_TOLERANCE

    def navigate_toward_goal(
        self,
        lidar_distance: float | None = None,
    ) -> None:
        """Drive straight forward toward the goal while front clearance is safe."""
        if self.goal_position is None:
            self.publish_stop()
            return
        if lidar_distance is None or math.isnan(lidar_distance):
            self.publish_stop()
            return
        if lidar_distance == -math.inf or lidar_distance < 0.0:
            self.publish_stop()
            self.get_logger().error(
                "LiDAR returned an invalid clearance; robot stopped."
            )
            return
        # handle_obstacle() can call this after the avoidance logic reports
        # GO_TO_GOAL, so re-check clearance here instead of assuming it.
        if lidar_distance <= self.OBSTACLE_DISTANCE_THRESHOLD:
            self.publish_stop()
            return

        command = Twist()
        # Approach slowly as the bumper clearance nears 13 cm to reduce
        # overshoot between LiDAR scans and control updates, but never drop
        # below the speed the base needs to actually move.
        speed = self.APPROACH_SPEED_GAIN * (
            lidar_distance - self.OBSTACLE_DISTANCE_THRESHOLD
        )
        command.linear.x = max(
            self.MIN_LINEAR_SPEED,
            min(self.MAX_LINEAR_SPEED, speed),
        )

        self.command_publisher.publish(command)

    def handle_obstacle(self) -> None:
        """Update obstacle state and stop when safe wall-follow data is absent."""

        if self.obstacle_avoidance is None:
            self.publish_stop()
            return

        avoidance_state = self.obstacle_avoidance.update_state()
        if avoidance_state is None:
            self.publish_stop()
            return

        self.navigation_state = avoidance_state
        if avoidance_state == "GO_TO_GOAL":
            self.obstacle_threshold_reported = False
            self.navigate_toward_goal(self.lidar.get_latest_distance())
            return

        # Front-only clearance cannot determine a safe wall-following direction.
        self.publish_stop()

    def publish_stop(self) -> None:
        """Publish a zero-velocity command."""
        self.command_publisher.publish(Twist())

    def shutdown_controller(self) -> None:
        """Stop the robot and release resources owned by this controller."""
        # After Ctrl+C the ROS context may already be shut down, so each step
        # is guarded to let the rest of the cleanup still run.
        try:
            self.publish_stop()
        except Exception as error:
            print(f"Could not publish final stop command: {error}")

        if self.control_timer is not None:
            try:
                self.control_timer.cancel()
                self.destroy_timer(self.control_timer)
            except Exception:
                pass
            self.control_timer = None

        for node in (self.lidar, self.odometry, self):
            try:
                node.destroy_node()
            except Exception:
                pass