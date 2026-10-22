## Main.py

"""
Starts the sensor-only TurtleBot 4 program. The nodes collect LiDAR and
odometry data.

The current program only collects and reports LiDAR and odometry data. It
prompts for a forward goal distance in meters at startup, but does not use it
for navigation or publish movement commands yet.
"""

import rclpy


def ask_for_goal_distance():
    """Prompt for a finite, non-negative goal distance in meters."""
    while True:
        try:
            # ask for a forward goal distance in meters
            # Note to the user to only input ONE number.
            # cast it as a float
            pass
        except ValueError:
            # tell the user to enter a valid number
            # then continue
            pass

        # Check if the distance is finite AND non-negative
            # import math
        # if it is return it.
        # otherwise, print an error message to be finit and non-negative.


def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create the controller
        # make sure to import this: from robot_controller import RobotController
    # Then get the goal distance from the user input 
    #   and store it in the controller as an attribute called goal_distance.

    # create a single-threaded executor to manage the nodes
    ## make sure to import this: from rclpy.executors import SingleThreadedExecutor
    # then add each node to the executor

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