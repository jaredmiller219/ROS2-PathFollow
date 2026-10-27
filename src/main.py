## Main.py

"""
Starts the sensor-only TurtleBot 4 program. The nodes collect LiDAR and
odometry data.

The current program only collects and reports LiDAR and odometry data. It
prompts for a forward goal distance in meters at startup, but does not use it
for navigation or publish movement commands yet.
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

    # Keep the sensor nodes running until interrupted.
    try:
        # Spin the executor (use .spin())
        pass

    except KeyboardInterrupt:
        # print that the program was interrupted by the user
        pass
        
    finally:
        # Shut down ROS 2 safely when the program ends 
        # (shutdown the executor).
        
        # destroy the nodes to free resources
        # first check if rclpy is ok, if so then shutdown rclpy
        pass

if __name__ == '__main__':
    main()