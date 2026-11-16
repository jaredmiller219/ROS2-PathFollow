#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

export DISPLAY="${DISPLAY:-:1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

world="${WORLD:-path_following}"

Xvfb "$DISPLAY" -ac -screen 0 "${VNC_RESOLUTION:-1600x900x24}" \
    >/tmp/xvfb.log 2>&1 &
sleep 2

openbox >/tmp/openbox.log 2>&1 &
x11vnc -display "$DISPLAY" -forever -nopw -shared -quiet \
    -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc "${NOVNC_PORT:-6080}" \
    localhost:5900 >/tmp/websockify.log 2>&1 &

exec ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
    "world:=${world}" "$@"
