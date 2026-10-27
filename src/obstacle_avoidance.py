## obstacle_avoidance.py

"""
Determines how the robot should navigate around detected obstacles.
Uses a simplified Bug algorithm to move around objects and return toward
the original path to continue toward the destination.
"""

# Integration references for the outline below:
# - In robot_controller.py, create/pass the LidarSensor instance from lidar.py
#   and the OdometryTracker instance from odometry.py into this logic.
# - Use LidarSensor.get_latest_distance() from lidar.py to get the closest
#   valid obstacle distance in the forward field of view, adjusted for the
#   sensor-to-bumper offset.
# - Use OdometryTracker.has_received_odometry from odometry.py to check that
#   position data has arrived before making a navigation decision.
# - Use OdometryTracker.get_position() from odometry.py to get the current
#   (x, y) position, and OdometryTracker.get_orientation() from odometry.py to
#   get the current heading.
# - The current LiDAR interface does not expose directional scan ranges to
#   obstacle_avoidance.py. Wall following that needs left/right ranges will
#   require LidarSensor to expose processed scan data first.

import math

from lidar import LidarSensor
from odometry import OdometryTracker


class ObstacleAvoidance:
    """Track Bug-style obstacle avoidance using the controller's sensor nodes."""

    def __init__(
        self,
        lidar: LidarSensor,
        odometry: OdometryTracker,
        start_position: tuple[float, float],
        goal_position: tuple[float, float],
    ):
        self.lidar = lidar
        self.odometry = odometry
        self.start_position = start_position
        self.goal_position = goal_position

        x_start, y_start = start_position
        x_goal, y_goal = goal_position

        self.line_a = y_start - y_goal
        self.line_b = x_goal - x_start
        self.line_c = (x_start * y_goal) - (x_goal * y_start)

        self.obstacle_hit_point = None
        self.m_line_tolerance = 0.1
        self.state = "GO_TO_GOAL"


    def distance_to_goal(self, current_position: tuple[float, float]) -> float:
        """Return the Euclidean distance from a position to the goal."""
        x_difference = self.goal_position[0] - current_position[0]
        y_difference = self.goal_position[1] - current_position[1]
        return math.hypot(x_difference, y_difference)

    def check_motion_line(self, current_position: tuple[float, float]) -> bool:
        """
        Return whether a position is within tolerance of the start-goal line.
        """
        x, y = current_position
        numerator = abs((self.line_a * x) + (self.line_b * y) + self.line_c)
        denominator = math.hypot(self.line_a, self.line_b)

        # A zero-length start-to-goal line has no direction or distinct M-line.
        if denominator == 0.0: return False

        distance_from_line = numerator / denominator

        # Check if this perpendicular distance is smaller than your threshold:
            # If it is smaller, return True (the robot is on the line)
            # Otherwise, return False (the robot is not on the line)
        return distance_from_line <= self.m_line_tolerance


    def robot_leave(self, current_position: tuple[float, float]) -> bool:
    # Define a function that accepts the robot's current (x, y) coordinates to decide when to leave:
        if self.obstacle_hit_point == None: return False
        # Check if a hit point was ever recorded:

            # If no hit point exists (None), return False immediately

        # Call your distance function to find how far the robot's current position is from the goal
        current_position_distance = self.distance_to_goal(current_position) 
        # Call your distance function to find how far the original obstacle hit point was from the goal
        hit_position = self.obstacle_hit_point(current_position) 
        # Check if the robot is currently crossing the M-line (Call your M-line check function)
        if self.check_motion_line(current_position) and (current_position_distance < hit_position): 
            return True
        
        else:
            return False
        # Check if the current distance to the goal is strictly less than the hit point's distance to the goal
        # Evaluate both checks together:
            # If BOTH conditions are met, return True (safe to leave the obstacle and head back to the goal)
            # If EITHER condition fails, return False (keep following the obstacle's boundary)


    # Define your main execution cycle or sensor callback function (e.g., handling laser/odometry data):
        # Check OdometryTracker.has_received_odometry from odometry.py before using its pose.
        # Retrieve the current position with OdometryTracker.get_position()
        # from odometry.py and heading with
        # OdometryTracker.get_orientation() from odometry.py.
        # Retrieve forward clearance with LidarSensor.get_latest_distance()
        # from lidar.py.
        # Treat missing or invalid readings as unavailable; do not infer that
        # the path is clear.
        
        # Check the current state of the robot state variable:
        
        # IF the state is "GO_TO_GOAL":
            # Check if your sensor readings detect an obstacle directly in front of the robot:
                # Save the current robot position into your hit point variable
                # Update the robot state variable to "WALL_FOLLOW"
                # Stop the forward progress and prepare to turn
            # OTHERWISE (path is clear):
                # Calculate the heading angle toward the goal and command velocities to drive straight
                
        # IF the state is "WALL_FOLLOW":
            # Call your leave condition function using the current robot position
            # Check if that function returns True:
                # Clear the stored hit point variable back to None
                # Update the robot state variable back to "GO_TO_GOAL"
                # Align the robot back toward the goal direction
            # OTHERWISE (leave function returns False):
                # Use directional ranges to find the object wall only after
                # LidarSensor exposes those ranges; its current public API
                # provides only the closest front-field distance.
                # Calculate steering velocities to maintain a safe, fixed distance from the object
                # Drive parallel to the object's boundary to navigate around it
    
    def update_state(self) -> str | None:
            """Update the avoidance state from current sensors; return None if unready."""
            if not self.odometry.has_received_odometry:
                return None

            lidar_distance = self.lidar.get_latest_distance()
            if (lidar_distance is None or math.isnan(lidar_distance) or lidar_distance == -math.inf):
                return None

            current_position = self.odometry.get_position()

            if self.state == "GO_TO_GOAL":
                if lidar_distance <= self.obstacle_distance_threshold:
                    self.obstacle_hit_point = current_position
                    self.state = "WALL_FOLLOW"
            
            elif self.state == "WALL_FOLLOW":
                if (self.obstacle_hit_point is not None
                    and self.check_motion_line(current_position)
                    and self.distance_to_goal(current_position)
                    < self.distance_to_goal(self.obstacle_hit_point)
                ):
                    self.obstacle_hit_point = None
                    self.state = "GO_TO_GOAL"
            
            else:
                raise ValueError(f"Unsupported obstacle-avoidance state: {self.state}")

            return self.state
