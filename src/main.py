## Main.py

"""
Starts the TurtleBot 4 obstacle avoidance program and coordinates
the robot controller, LiDAR, odometry, and navigation system.
"""

import rclpy
from obstacle_avoidance import RobotController

def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # create an instance of the robot controller node
    robot_controller = RobotController()

    # Keep the program running while the robot navigates
    try: rclpy.spin(robot_controller)
    except KeyboardInterrupt: pass
        
    finally:
        # Shut down ROS 2 safely when the program ends
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()