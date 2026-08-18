#!/usr/bin/env bash
# Combine the last >=3 single-scene results into one multi-scene extrinsic.
#
#   docker/run_multi.sh <cam>
#
# Prerequisite: run docker/run.sh <cam> sceneN for at least 3 scenes first.
# Each single-scene run appends to output/<cam>/circle_center_record.txt, which
# this step reads (it uses the last 3 scene blocks). Results are written to
# output/<cam>/multi_calib_result.txt and multi_calib_extrinsics.yaml.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="fast-calib2:noetic"
CAM="${1:-cam0}"

docker run --rm --net=host \
    -v "${REPO}:/w" \
    "${IMAGE}" \
    /w/docker/.ws_build/multi_fast_calib \
      --config "/w/config/cameras/${CAM}.yaml" \
      --output "/w/output/${CAM}"
