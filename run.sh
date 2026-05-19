#!/usr/bin/env bash
# Launch the OHM + ROS 2 Humble container.
# Host networking + GPU passthrough by default so it can consume docker-stereo
# topics and use CUDA for ray integration.
#
# Usage:
#   ./run.sh                                          # interactive shell
#   ./run.sh ros2 launch ohm_ros2 ohm_live.launch.py  # auto-launch node
#   DATA=/path/to/bags ./run.sh                        # different /data mount
#   GPU=0 ./run.sh                                     # disable GPU (dev only)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_TAG="${TAG:-ohm-ros2:humble}"
DATA_DIR="${DATA:-${SCRIPT_DIR}/../../bags}"
WS_DIR="${WS:-${SCRIPT_DIR}/user_ws}"
CONTAINER_NAME="${NAME:-ohm-ros2}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

DOCKER_ARGS=(
    --rm
    -it
    --name "${CONTAINER_NAME}"
    --hostname ohm-ros2
    --network host
    --ipc host
    --pid host
    -e "TERM=${TERM:-xterm-256color}"
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    -e "RMW_IMPLEMENTATION=${RMW}"
)

# Workspace bind mount (always — ohm_ros2 is not baked into the image).
DOCKER_ARGS+=(-v "${WS_DIR}:/root/user_ws")
echo "==> Mounting ${WS_DIR} -> /root/user_ws (rw)"

# Data mount (for saving .ohm files, reading bags, etc.)
if [[ -d "${DATA_DIR}" ]]; then
    DOCKER_ARGS+=(-v "${DATA_DIR}:/data")
    echo "==> Mounting ${DATA_DIR} -> /data (rw)"
fi

# GPU passthrough — enabled by default, disable with GPU=0 for CPU-only shells.
if [[ "${GPU:-1}" == "1" ]]; then
    DOCKER_ARGS+=(--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics)
    echo "==> GPU passthrough enabled"
fi

# X11 forwarding for RViz / visualization tools.
if [[ -n "${DISPLAY:-}" ]]; then
    DOCKER_ARGS+=(
        -e "DISPLAY=${DISPLAY}"
        -e "QT_X11_NO_MITSHM=1"
        -v /tmp/.X11-unix:/tmp/.X11-unix
    )
    echo "==> X11 forwarding on ${DISPLAY}"
fi

echo "==> Launching ${IMAGE_TAG}"
exec docker run "${DOCKER_ARGS[@]}" "${IMAGE_TAG}" "$@"
