#!/usr/bin/env bash
# Build the FAST-Calib2 Docker image and compile the (ROS-free) binaries with
# plain CMake. Compiled binaries land in docker/.ws_build (bind-mounted) so
# they survive across `docker run --rm`.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="fast-calib2:noetic"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[build] building image ${IMAGE}"
  docker build -t "${IMAGE}" "${REPO}/docker"
fi

mkdir -p "${REPO}/docker/.ws_build"

echo "[build] compiling (cmake, no ROS)"
docker run --rm \
    -v "${REPO}:/w" \
    "${IMAGE}" \
    bash -lc "cd /w/docker/.ws_build && cmake -DCMAKE_BUILD_TYPE=Release /w && make -j\$(nproc)"

echo "[build] done. Binaries in docker/.ws_build/. Run: docker/run.sh <cam> <scene>"
