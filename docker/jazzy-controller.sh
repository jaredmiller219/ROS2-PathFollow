#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
cd /workspace

exec python3 src/main.py --ros-args -r /cmd_vel:=/cmd_vel_unstamped
