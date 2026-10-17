# ROS2-PathFollow

Autonomous TurtleBot 4 navigation using LiDAR and odometry to detect, avoid
obstacles, and reach a target destination.

## Run the development environment

The project includes a Docker Compose environment so contributors can use the
same ROS 2 setup without installing ROS 2 or using VS Code Dev Containers.

### Requirements

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) on macOS or
  Windows, or Docker Engine with the Docker Compose plugin on Linux.
- An internet connection for the initial image build.

### Start a shell

From the repository directory, run:

```sh
docker compose -f docker/compose.yaml run --build --rm ros2-pathfollow
```

This builds the ROS 2 image the first time and opens a shell with the ROS 2
environment sourced. The repository is mounted into the container at
`/workspace`, so edits on the host are available there. Type `exit` to leave.

You can run a command in the container instead of opening a shell, for example:

```sh
docker compose -f docker/compose.yaml run --build --rm ros2-pathfollow python3 src/main.py
```

The navigation modules are currently scaffolding; `src/main.py` does not yet
start robot navigation. A TurtleBot 4 and its ROS 2 network configuration are
also required to test against a physical robot.

## VS Code Dev Containers (optional)

If you prefer VS Code, open this repository and choose **Reopen in Container**.
The Dev Container uses the same Compose service and Docker image as the
standalone instructions above.
