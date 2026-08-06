#!/usr/bin/env bash
# Combine the last >=3 single-scene results into one multi-scene extrinsic.
#
#   docker/run_multi.sh <cam>
#
# Prerequisite: run docker/run.sh <cam> sceneN for at least 3 scenes first.
# Each single-scene run appends to output/<cam>/circle_center_record.txt, which
# this step reads (it uses the last 3 scene blocks). Result is written to
# output/<cam>/multi_calib_result.txt.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="fast-calib2:noetic"
CAM="${1:-cam0}"

docker run --rm -it --net=host \
    -v "${REPO}:/root/calib_ws/src/fast_calib" \
    -v "${REPO}/docker/.ws_build:/root/calib_ws/build" \
    -v "${REPO}/docker/.ws_devel:/root/calib_ws/devel" \
    "${IMAGE}" \
    bash -lc "roslaunch fast_calib multi_calib_cam.launch cam:=${CAM}"
