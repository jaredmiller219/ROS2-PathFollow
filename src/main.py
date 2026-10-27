## Main.py

"""
Starts the TurtleBot 4 navigation program. The executor runs the controller,
LiDAR, and odometry nodes.

The controller publishes movement commands while navigating toward the
requested forward goal distance.
"""

import math

import rclpy
from rclpy.executors import SingleThreadedExecutor

from robot_controller import RobotController


def ask_for_goal_distance():
    """Prompt for a finite, non-negative goal distance in meters."""
    while True:
        try:
            # ask for a forward goal distance in meters
            goal_input = input("Input the goal distance (must be ONE number): ")
            goal_distance = float(goal_input)
        except ValueError:
            print("Please enter one valid number.")
            continue

        if math.isfinite(goal_distance) and goal_distance >= 0:
            return goal_distance

        print("Please enter a finite, non-negative distance.")


def main(args=None):
    robot_controller = None
    executor = None
    try:
        goal_distance = ask_for_goal_distance()

        # Initialize ROS only after obtaining valid user input.
        rclpy.init(args=args)

        robot_controller = RobotController(goal_distance)
        executor = SingleThreadedExecutor()
        nodes = (
            robot_controller.lidar,
            robot_controller.odometry,
            robot_controller,
        )
        for node in nodes:
            executor.add_node(node)

        robot_controller.start()
        executor.spin()

    except KeyboardInterrupt:
        print("Navigation interrupted by user.")

    finally:
        if executor is not None:
            executor.shutdown()
        if robot_controller is not None:
            robot_controller.shutdown_controller()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()