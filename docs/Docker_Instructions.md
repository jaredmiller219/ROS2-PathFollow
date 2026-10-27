# Docker

## Run the development environment

Choose either Docker Compose from a terminal or VS Code Dev Containers. Both are set up to use
the ROS 2 Lyrical image and mount this repository at `/workspace`.

### Requirements

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) on macOS or
  Windows, or Docker Engine with the Compose plugin on Linux.
- For the VS Code option: [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
- An internet connection to download the ROS image the first time.

## Docker Compose

### Start an interactive shell

Make sure Docker is running. From the repository directory, build the image if
needed and start an interactive shell:

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

### Test the LiDAR node

Run the LiDAR subscriber on its own:

```sh
docker compose -f docker/compose.yaml run --build --rm ros2-pathfollow python3 src/lidar.py
```

The node logs when it has subscribed to `/scan` and prints the nearest
front-obstacle distance whenever a scan message arrives. If it only reports
that it is waiting, make sure a ROS 2 publisher is sending `sensor_msgs/msg/LaserScan`
messages on `/scan` and that the node can communicate with that publisher.

## VS Code - Dev Containers

### Enable or disable detection

To disable Dev Container detection, add `.disabled` to the config filename:
`.devcontainer/devcontainer.json.disabled`. Remove `.disabled` from the
filename to enable detection again.

### Configure the container

If the sample file is not present, create `.devcontainer/devcontainer.json` and
put this configuration in it:

```json
{
  "name": "ROS 2 PathFollow Dev Environment",
  "image": "ros:lyrical",
  "workspaceFolder": "/workspace",
  "workspaceMount": "source=${localWorkspaceFolder},target=/workspace,type=bind",
  "runArgs": [
    "--name", "ros2_pathfollow"
  ],
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-iot.vscode-ros",
        "ms-python.python"
      ]
    }
  },
  "postCreateCommand": "echo 'source /opt/ros/lyrical/setup.bash' >> ~/.bashrc"
}
```

### Open the repository in the container

Open this repository in VS Code, then run **Dev Containers: Reopen in
Container** from the Command Palette.

VS Code pulls the `ros:lyrical` image if needed, mounts the repository at
`/workspace`, and installs the listed extensions. The container is named
`ros2_pathfollow`; if that name is already in use, remove the existing container
or change the name in `runArgs`.
