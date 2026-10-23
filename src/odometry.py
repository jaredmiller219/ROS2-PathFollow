## odometry.py

"""
Tracks the robot's estimated position, orientation, and distance traveled
using odometry data. Helps the robot determine its progress toward the
destination and return to its original path after avoiding obstacles.
"""

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryTracker(Node):
    def __init__(self):
        """
        Initialize node and subscribe to odometry topic.
        """
        super().__init__('odometry_tracker')

        # State variables
        self.position = (0.0, 0.0)
        self.orientation = 0.0
        self.distance_traveled = 0.0
        self.last_position = None
        self.has_received_odometry = False
        
        # Subscribe to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )

    def get_position(self):
        """
        Return current robot position.
        """
        return self.position
        
    def get_orientation(self):
        """
        Return current robot orientation.
        """
        return self.orientation
    
    def get_distance_traveled(self):
        """
        Return total distance traveled.
        """
        return self.distance_traveled
    
    def _calculate_distance(self, pos1, pos2):
        """
        Calculate Euclidean distance between two positions.
        """
        # Euclidean distance formula: sqrt((x2 - x1)^2 + (y2 - y1)^2)
        return math.sqrt(
            math.pow(pos1[0] - pos2[0], 2) + 
            math.pow(pos1[1] - pos2[1], 2)
        )
    
    def _extract_orientation(self, msg: Odometry):
        """
        Extract orientation angle from odometry message.
        """
        # get the quaternion, by using msg.pose.pose.orientation
        # sin yaw is calculated as 2*(w*z + x*y)
        # cos yaw is calculated as 1 - 2*(y^2 + z^2)
        # then return the arc tangent with both sin and cos to get the yaw angle
            ## arctan2(sin_yaw, cos_yaw)


    def odometry_callback(self, msg: Odometry):
        """
        Receive odometry data and update position tracking.
        """
        # Extract position from message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.orientation = self._extract_orientation(msg)
        
        # First-frame guard
        if self.last_position is None:
            self.last_position = (x, y)
            self.position = (x, y)
            self.has_received_odometry = True
            return
        
        # Calculate step distance safely now that last_position is valid
        self.distance_traveled += self._calculate_distance(self.last_position, (x, y))
        
        # Update states for the next iteration
        self.position = (x, y)
        self.last_position = (x, y)