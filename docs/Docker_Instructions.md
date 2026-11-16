# Docker

## Run the development environment

Choose either Docker Compose from a terminal or VS Code Dev Containers. Both
use the ROS 2 Lyrical image and mount the repository root at `/workspace`.
The Docker Compose files are in `docker/`; the optional Dev Container
configuration is `.devcontainer/devcontainer.json.disabled`.

### Requirements

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) on macOS or
  Windows, or Docker Engine with the Compose plugin on Linux.
- For the VS Code option: [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
- An internet connection to download the ROS image the first time.

## Docker Compose

### Start an interactive shell

Make sure Docker is running. From the repository root (the directory containing
`docker/` and `src/`), build the image if needed and start an interactive shell:

```sh
docker compose -f docker/compose.yaml run --build --rm ros2-pathfollow
```

The ROS 2 environment is sourced automatically, and edits in the repository are
available inside the container. Type `exit` to leave; `--rm` removes the
container when it exits.

### Run the project entry point

To run the project entry point without opening a shell:

```sh
docker compose -f docker/compose.yaml run --build --rm ros2-pathfollow python3 src/main.py
```

This runs `src/main.py`, which starts the robot controller. The controller
subscribes to `/scan` and `/odom` and publishes movement commands to `/cmd_vel`.
The scan is processed by the occupancy grid for obstacle avoidance.

## TurtleBot 4 Gazebo simulation (ROS 2 Jazzy)

The default `ros2-pathfollow` service remains on ROS 2 Lyrical. For a visual
simulation, use the `sim` profile, which builds `docker/Dockerfile.sim` from
`ros:jazzy` with Gazebo Harmonic and the official TurtleBot 4 simulator
packages.

### Start the simulator

From the repository root:

```sh
docker compose -f docker/compose.yaml --profile sim up -d --build turtlebot4-sim
```

Open `http://localhost:6080/vnc.html?autoconnect=true&resize=scale` in a browser.
The first build downloads Gazebo and TurtleBot 4 packages and can take a while.
Gazebo renders in software inside the container, so expect lower FPS on macOS.

The simulator launches `turtlebot4_gz.launch.py` with `WORLD=path_following`
by default. Compose bind-mounts `sim/worlds/path_following.sdf` into Gazebo's
resource path as `path_following.sdf`, and the file's `<world name>` is
`path_following` so the TurtleBot 4 Jazzy sensor bridges publish `/scan`,
`/odom`, and related topics correctly. The included world places a 0.4 m
obstacle about 1.2 m ahead of the spawn point; use a goal distance above
roughly 2 m to exercise obstacle avoidance.

`WORLD` must be a Gazebo world *name* (the `<world name>` inside the SDF), not
a filesystem path. To use the built-in warehouse world instead:

```sh
WORLD=warehouse docker compose -f docker/compose.yaml --profile sim up -d --force-recreate --build turtlebot4-sim
```

### Run the controller against the simulator

In a second terminal:

```sh
docker compose -f docker/compose.yaml --profile sim run --rm ros2-pathfollow-jazzy
```

This runs `src/main.py` on ROS 2 Jazzy and remaps `/cmd_vel` to
`/cmd_vel_unstamped`. Jazzy TurtleBot 4 expects `TwistStamped` on `/cmd_vel`; the
simulator's motion-control node accepts unstamped `Twist` on
`/cmd_vel_unstamped`.

Enter the goal distance when prompted. Watch Gazebo through the noVNC page for
movement, and use the following for simulator logs:

```sh
docker compose -f docker/compose.yaml --profile sim logs -f turtlebot4-sim
```

### Stop the simulator

```sh
docker compose -f docker/compose.yaml --profile sim stop turtlebot4-sim
```

If port 6080 is already in use, set `NOVNC_HOST_PORT` before `up`, for example
`NOVNC_HOST_PORT=6081`, then open the same noVNC URL on port 6081.

## VS Code - Dev Containers

### Enable Dev Container detection

The configuration is provided at
`.devcontainer/devcontainer.json.disabled` and is disabled by default. From the
repository root, rename it to `.devcontainer/devcontainer.json` to enable VS
Code Dev Container detection. Rename it back to add `.disabled` and disable
detection.

### Configure the container

The disabled sample already contains the configuration. If you need to change
it, edit `.devcontainer/devcontainer.json` after renaming the file.

### Open the repository in the container

Open this repository in VS Code, then run **Dev Containers: Reopen in
Container** from the Command Palette.

VS Code starts a container from `ros:lyrical`, mounts the repository root at
`/workspace`, and installs the listed extensions. The container is named
`ros2_pathfollow`; if that name is already in use, remove the existing
container or change the name in `runArgs`.
