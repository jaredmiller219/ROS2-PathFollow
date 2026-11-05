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
