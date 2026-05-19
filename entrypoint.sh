#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

WS=/root/user_ws

if [ -d "${WS}/src" ] && [ -n "$(ls -A ${WS}/src 2>/dev/null)" ]; then
    if [ ! -f "${WS}/install/setup.bash" ]; then
        echo "[entrypoint] First-time build of user_ws ..."
        cd "${WS}"
        colcon build --symlink-install 2>&1 | tail -10
    fi
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

exec "$@"
