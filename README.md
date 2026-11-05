# ROS2-PathFollow

Autonomous TurtleBot 4 navigation using LiDAR and odometry to detect, avoid
obstacles, and reach a target destination.

## Navigation flow

The controller projects each `/scan` measurement into a sparse 2D occupancy
grid in the odometry coordinate frame. It drives toward the requested distance
while the path is clear. When an obstacle is close in front, it checks mapped
clearance on either side, moves aside, passes the obstacle, and returns to the
original path before continuing to the destination. If neither bypass route is
clear, the robot stops.

The occupancy grid is refreshed from each LiDAR scan; it is a local navigation
map, not a persistent SLAM map. The original `lidar.py` distance processing is
left unchanged.

## Development environment

Use Docker Compose from a terminal or open the project in VS Code with Dev
Containers. See the [Docker Instructions Guide](docs/Docker_Instructions.md)
for requirements and setup instructions.

A TurtleBot 4 and its ROS 2 network configuration are also required to test
against a physical robot.

Run the unit tests in the ROS 2 environment with:

```sh
PYTHONPATH=src python3 -m pytest -q
```
