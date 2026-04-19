#!/usr/bin/env bash
# Build the OHM + ROS 2 Humble image.
# Usage:
#   ./build.sh                   # default: ohm-ros2:humble, OHM master
#   ./build.sh v0.5.0            # pin OHM to a git tag/commit
#   TAG=ohm-ros2:dev ./build.sh  # custom image tag
#   JOBS=8 ./build.sh            # override parallel build jobs
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

IMAGE_TAG="${TAG:-ohm-ros2:humble}"
OHM_REF="${1:-master}"
JOBS="${JOBS:-$(nproc)}"

echo "==> Building ${IMAGE_TAG}"
echo "    OHM ref:   ${OHM_REF}"
echo "    Jobs:      ${JOBS}"

docker build \
    --tag "${IMAGE_TAG}" \
    --build-arg "OHM_REF=${OHM_REF}" \
    --build-arg "BUILD_JOBS=${JOBS}" \
    --progress=plain \
    .

echo "==> Built ${IMAGE_TAG}"
docker image ls "${IMAGE_TAG%%:*}"
