#!/usr/bin/env bash
# Run a single-scene LiDAR-camera calibration inside the container.
#
#   docker/run.sh <cam> <scene> [extra fast_calib args...]
#
# Examples:
#   docker/run.sh cam0 scene1
#   docker/run.sh cam2 scene3 --debug-dir output/cam2/debug_scene3
#
# The binary prints "T_cam_lidar" + RMSE and exits. Results -> output/<cam>/.
# Scene data: calib_data/<cam>/<scene>/ with image.png plus record/ (cyber
# record files) or cloud.pcd; a cloud_roi.txt in the scene dir is auto-applied.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="fast-calib2:noetic"

CAM="${1:-cam0}"
SCENE="${2:-scene1}"
shift 2 2>/dev/null || true

docker run --rm --net=host \
    -v "${REPO}:/w" \
    "${IMAGE}" \
    /w/docker/.ws_build/fast_calib \
      --config "/w/config/cameras/${CAM}.yaml" \
      --scene "/w/calib_data/${CAM}/${SCENE}" \
      --output "/w/output/${CAM}" "$@"
