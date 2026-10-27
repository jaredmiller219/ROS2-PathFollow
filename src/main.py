## Main.py

"""
Starts the TurtleBot 4 navigation program. The executor runs the controller,
LiDAR, and odometry nodes.

The controller publishes movement commands while navigating toward the
requested forward goal distance.
"""

import math

from robot_controller import RobotController


def ask_for_goal_distance():
    """Prompt for a finite, non-negative goal distance in meters."""
    while True:
        try:
            # ask for a forward goal distance in meters
            goal_input = input("Input the goal distance (must be ONE number): ")
            goal_distance = float(goal_input)
        except ValueError:
            continue

        if math.isfinite(goal_distance) and goal_distance >= 0:
            return goal_distance


def main(args=None):
    goal_distance = ask_for_goal_distance()
    RobotController.run(goal_distance, args=args)


if __name__ == '__main__':
    main()