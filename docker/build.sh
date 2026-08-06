#!/usr/bin/env bash
# Build the FAST-Calib2 Docker image and compile the catkin workspace.
# Compiled binaries land in docker/.ws_devel (bind-mounted) so they survive
# across `docker run --rm`.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="fast-calib2:noetic"

echo "[build] building image ${IMAGE}"
docker build -t "${IMAGE}" "${REPO}/docker"

mkdir -p "${REPO}/docker/.ws_build" "${REPO}/docker/.ws_devel"

echo "[build] compiling catkin workspace"
docker run --rm \
    -v "${REPO}:/root/calib_ws/src/fast_calib" \
    -v "${REPO}/docker/.ws_build:/root/calib_ws/build" \
    -v "${REPO}/docker/.ws_devel:/root/calib_ws/devel" \
    "${IMAGE}" \
    bash -lc "cd /root/calib_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j\$(nproc)"

echo "[build] done. Run a calibration with: docker/run.sh <cam> <scene>"
