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
   obstacle blocks the path corridor, it measures the obstacle's lateral
   extent and selects the clearer side.

4. `RobotController` executes the bypass as separate legs: turn aside, drive
   sideways clear of the obstacle, turn toward the goal direction, drive
   forward past it, turn back toward the path line, return to the line, and
   turn toward the goal again. If no clear bypass is available, it stops.

## Development environment

Use Docker Compose from a terminal or open the project in VS Code with Dev
Containers. See the [Docker Instructions Guide](docs/Docker_Instructions.md)
for requirements and setup instructions.

A TurtleBot 4 and its ROS 2 network configuration are also required to test
against a physical robot.
