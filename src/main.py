## Main.py

"""Read a forward goal distance and start TurtleBot navigation."""

import math

from robot_controller import RobotController


def ask_for_goal_distance() -> float:
    """Prompt until the user enters a finite, non-negative distance."""
    while True:
        try:
            goal_distance = float(input("Goal distance in meters: "))
        except ValueError:
            print("Enter one valid number.")
            continue

        if math.isfinite(goal_distance) and goal_distance >= 0.0:
            return goal_distance
        print("Distance must be finite and non-negative.")


def main(args=None) -> None:
    RobotController.run(ask_for_goal_distance(), args=args)


if __name__ == '__main__':
    main()