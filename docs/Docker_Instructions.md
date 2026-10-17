## Run the development environment

Choose either Docker Compose from a terminal or VS Code Dev Containers. Both use
the ROS 2 Lyrical image and mount this repository at `/workspace`.

### Requirements

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) on macOS or
  Windows, or Docker Engine with the Compose plugin on Linux.
- For the VS Code option: [VS Code](https://code.visualstudio.com/) and the
  **Dev Containers** extension.
- An internet connection to download the ROS image the first time.

### Run with Docker Compose

Make sure Docker is running. From the repository directory, build the image if
needed and start an interactive shell:

```sh
docker compose -f docker/compose.yaml run --build --rm ros2-pathfollow
```

The ROS 2 environment is sourced automatically, and edits in the repository are
available inside the container. Type `exit` to leave; `--rm` removes the
container when it exits.

To run the project entry point without opening a shell:

```sh
docker compose -f docker/compose.yaml run --build --rm ros2-pathfollow python3 src/main.py
```

### Run with VS Code Dev Containers

In the repository, create a `.devcontainer` folder if it does not exist, then
create `.devcontainer/devcontainer.json` inside it. Put this configuration in
that file:

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

Open this repository in VS Code, then run **Dev Containers: Reopen in
Container** from the Command Palette.

VS Code pulls the `ros:lyrical` image if needed, mounts the repository at
`/workspace`, and installs the listed extensions. The container is named
`ros2_pathfollow`; if that name is already in use, remove the existing container
or change the name in `runArgs`. Use the integrated terminal to run project
commands, for example `python3 src/main.py`.

A TurtleBot 4 and its ROS 2 network configuration are also required to test
against a physical robot. The navigation modules are currently scaffolding;
`src/main.py` does not yet start robot navigation.
