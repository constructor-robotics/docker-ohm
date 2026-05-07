#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

WS=/root/user_ws
if [ -d "${WS}/src" ] && [ -n "$(ls -A ${WS}/src 2>/dev/null)" ]; then
    echo "[entrypoint] Building colcon workspace at ${WS} ..."
    cd "${WS}"
    colcon build --symlink-install
    echo "[entrypoint] Workspace built."
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

exec "$@"
