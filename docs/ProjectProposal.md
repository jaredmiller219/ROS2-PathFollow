# Introduction 

The goal of this project is to program a TurtleBot 4 to travel from a starting point to a destination while avoiding obstacles in its path. The robot will use LiDAR to detect objects, such as water bottles and backpacks, and wheel encoders and/or an IMU to estimate its movement and direction. Using a simplified Bug algorithm, the robot will navigate around obstacles and return to its original path when it is safe. It will also account for the LiDAR sensor’s position on the robot to maintain a safe distance from obstacles. 

This project is useful because it demonstrates the fundamental principles of autonomous navigation, combining sensors, motion control, and decision-making to allow a robot to operate without constant human guidance. Starting with simple obstacles and a known destination will make the project easier to develop and test. 

# Problem formulation 

The task of this project is to program a TurtleBot 4 to autonomously navigate from a starting point to a specified destination while avoiding obstacles in its path. The robot should follow a straight-line path whenever possible, but when an obstacle, such as a water bottle, backpack, or box, blocks its route, it should navigate around it and return toward its original path once the obstacle has been passed. The main goal is for the robot to reach its destination without colliding with any obstacles or requiring human intervention during navigation. 

Use the IMU and Lidar sensor, which will allow the robot to determine whether its path is clear and adjust its movement when necessary. Lidar distances are corrected by subtracting the sensor-to-front offset so clearance is measured from the robot's body. 

We will use the Bug algorithm. The robot drives straight toward the goal. When the lidar detects an obstacle within a set distance, it follows the obstacle's edge until it rejoins the start-to-goal line closer to the goal, then continues straight. The robot will maintain a safe distance from obstacles, accounting for its size and sensor placement. This repeats until the robot reaches the goal. We will use a basic movement and distance sensing before integrating obstacle avoidance and path recovery, making it easier to test and improve each part of the system independently. 

# Evaluation Method 

We will test on the TurtleBot 4 with start and goal points marked 8-10ft apart on a flat indoor floor. Scenarios (5 runs each): 

1.  No obstacles 

2.  One water bottle on the line 

3.  One backpack on the line 

4.  Water bottle and backpack on the line 

A run succeeds if the robot stops within a specific distance of the goal, never touches an obstacle, and needs no manual help. We will record the success rate and final distance from the goal for each scenario. The project meets its goals if the robot succeeds in at least 4 of 5 runs per scenario. 

