## odometry.py

"""
Tracks the robot's estimated position, orientation, and distance traveled
using odometry data. Helps the robot determine its progress toward the
destination and return to its original path after avoiding obstacles.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

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
        self.last_position = (0.0, 0.0)
        
        # Subscribe to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )
    
    def odometry_callback(self, msg: Odometry):
        """
        Receive odometry data and update position tracking.
        """
        # Extract position from message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate distance traveled
        self.distance_traveled += self._calculate_distance(
            self.last_position, (x, y)
        )
        
        # Update state
        self.position = (x, y)
        self.last_position = (x, y)
        self.orientation = self._extract_orientation(msg)
    
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
        pass
    
    def _extract_orientation(self, msg: Odometry):
        """
        Extract orientation angle from odometry message.
        """
        pass