## lidar.py

"""
Reads LiDAR sensor data from the TurtleBot 4 to detect obstacles and
measure their distance from the robot. Provides distance information
that the navigation system uses to determine whether the path is clear.
"""

import numpy as np
from sensor_msgs.msg import LaserScan

# Define constants at the top of your script or class
FOV_ANGLE = 30.0  # Total FOV angle in degrees (e.g., ±15 degrees)
SENSOR_TO_ROBOT_FRONT_BUMPER_DISTANCE = 0.05  # Distance in meters from sensor to front bumper


def scan_callback(self, msg: LaserScan) -> float:
    # 2. Process the LiDAR measurements.
    # a. Extract distances at different angles.
    ranges = np.array(msg.ranges)
    angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

    # b. Focus on the area in front of the robot.
    theta_max = np.radians(FOV_ANGLE / 2.0)

    # c. Ignore invalid or out-of-range measurements (lambda helper function).
    is_valid_range = lambda d: np.isfinite(d) & (d > 0.0)

    # 3. Determine obstacle distances.
    # a1. Filter the LiDAR scan to a front-facing field of view (FOV) to focus only on a ±X° forward arc.
    fov_mask = (angles >= -theta_max) & (angles <= theta_max)
    d_fov = ranges[fov_mask]

    # a2. Apply the filter defined in 2c that ignores invalid readings.
    valid_mask = is_valid_range(d_fov)
    d_valid = d_fov[valid_mask]

    # a3. Using the final filtered values, calculate the distance to the closest object in front of the robot.
    d_closest_object = np.min(d_valid) if len(d_valid) > 0 else float("inf")

    # b. Calculate the actual physical distance from the front bumper to the object to account for sensor offset[cite: 1].
    Distance_With_Sensor_Offset = (
        d_closest_object - SENSOR_TO_ROBOT_FRONT_BUMPER_DISTANCE
    )

    # c. Provide the processed distance data to obstacle_avoidance.py.
    return Distance_With_Sensor_Offset

print("LiDAR module loaded successfully.")