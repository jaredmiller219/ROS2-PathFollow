# ROS2-PathFollow

Autonomous TurtleBot 4 navigation using LiDAR and odometry to detect, avoid
obstacles, and reach a target destination.

## Navigation flow

1. `RobotController` subscribes to `/scan`, while `OdometryTracker` subscribes
   to `/odom`. The ROS executor processes both subscriptions and the controller's
   periodic movement timer.
   
2. After odometry is available, the controller sends each scan and the current
   robot pose to `OccupancyGrid`. The grid filters invalid/out-of-range returns
   and projects the remaining points into odometry (world) coordinates. Its
   default resolution is 0.05 m (5 cm) per cell; returns in the same cell are
   grouped, while obstacle clearance is measured from the stored world-coordinate
   points. Change `RobotController.OCCUPANCY_GRID_RESOLUTION` in
   `src/robot_controller.py` to adjust the cell width. Because this local
   navigation map is rebuilt from each LiDAR scan, a persistent SLAM map is not
   needed.

3. `ObstacleAvoidance` checks the mapped points ahead of the robot. If an
   obstacle is within the trigger distance, it checks candidate bypass routes
   for sufficient clearance.

4. The controller follows the selected waypoints by publishing velocity
   commands to `/cmd_vel`. It returns to the original path after passing the
   obstacle; if no clear bypass is available, it stops.

## Development environment

Use Docker Compose from a terminal or open the project in VS Code with Dev
Containers. See the [Docker Instructions Guide](docs/Docker_Instructions.md)
for requirements and setup instructions. For a visual Gazebo/TurtleBot 4
simulation in Docker, see the
[TurtleBot 4 Gazebo simulation](docs/Docker_Instructions.md#turtlebot-4-gazebo-simulation-ros-2-jazzy)
section.

A TurtleBot 4 and its ROS 2 network configuration are also required to test
against a physical robot.