## Main.py

"""
Starts the TurtleBot 4 navigation program. The executor runs the controller,
LiDAR, and odometry nodes.

The controller publishes movement commands while navigating toward the
requested forward goal distance.
"""

import math
from robot_controller import RobotController
from rclpy.executors import SingleThreadedExecutor
import rclpy


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
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    goal_distance = ask_for_goal_distance()
    robot_controller = RobotController(goal_distance)

    # create a single-threaded executor to manage the nodes
    # then add each node to the executor
    executor = SingleThreadedExecutor()
    executor.add_node(robot_controller)
    executor.add_node(robot_controller.lidar)
    executor.add_node(robot_controller.odometry)

    try:
        executor.spin()

    except KeyboardInterrupt:
        print("Navigation interrupted by user.")
        
    finally:
        executor.shutdown()
        robot_controller.shutdown_controller()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()