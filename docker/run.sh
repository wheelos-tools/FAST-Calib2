#!/usr/bin/env bash
# Run a single-scene LiDAR-camera calibration inside the container.
#
#   docker/run.sh <cam> <scene> [rviz]
#
# Examples:
#   docker/run.sh cam0 scene1          # calibrate cam0 against the Vanjee LiDAR
#   docker/run.sh cam2 scene3 true     # with RViz (needs X11, see docker/README.md)
#
# The node prints "T_cam_lidar" and RMSE, then enters a publish loop for RViz;
# press Ctrl-C once the result is printed. Results are written to output/<cam>/.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="fast-calib2:noetic"

CAM="${1:-cam0}"
SCENE="${2:-scene1}"
RVIZ="${3:-false}"

X11_ARGS=()
if [ "${RVIZ}" = "true" ]; then
    X11_ARGS=(-e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi

docker run --rm -it --net=host \
    -v "${REPO}:/root/calib_ws/src/fast_calib" \
    -v "${REPO}/docker/.ws_build:/root/calib_ws/build" \
    -v "${REPO}/docker/.ws_devel:/root/calib_ws/devel" \
    "${X11_ARGS[@]}" \
    "${IMAGE}" \
    bash -lc "roslaunch fast_calib calib_cam.launch cam:=${CAM} scene:=${SCENE} rviz:=${RVIZ}"
