#!/usr/bin/env bash
# Build FAST-Calib2 inside the Apollo dev environment (bazel, no ROS).
#
# Syncs the sources into <apollo workspace>/modules/calibration/fast_calib,
# builds with the workspace's bazel + third-party modules (pcl/opencv/eigen)
# inside the running Apollo dev container, and copies the binaries back to
# <repo>/build/.
#
# Configure via env:
#   APOLLO_HOST=/path/to/apollo      # host path of the Apollo workspace
#                                     (mounted as /apollo in the container)
#   APOLLO_C=<container>              # running Apollo dev container
#                                     (default: first apollo_dev_* found)
#   APOLLO_USER=nvidia                # user to build as inside the container
#
# Usage:  scripts/apollo_build.sh
set -euo pipefail

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$TOOLS/.." && pwd)"

APOLLO_HOST="${APOLLO_HOST:?set APOLLO_HOST=<host path of the Apollo workspace>}"
APOLLO_C="${APOLLO_C:-$(docker ps --format '{{.Names}}' | grep -m1 apollo_dev || true)}"
[ -n "$APOLLO_C" ] || { echo "ERROR: no running apollo_dev container; set APOLLO_C" >&2; exit 1; }
APOLLO_USER="${APOLLO_USER:-nvidia}"

PKG_HOST="$APOLLO_HOST/modules/calibration/fast_calib"
PKG="//modules/calibration/fast_calib"

echo "[apollo-build] sync sources -> $PKG_HOST"
mkdir -p "$PKG_HOST"
rsync -a --delete \
  --include='/BUILD' --include='/src/***' --include='/include/***' \
  --include='/proto/***' --include='/config/***' \
  --exclude='*' \
  "$REPO/" "$PKG_HOST/"

echo "[apollo-build] bazel build $PKG:all  (container $APOLLO_C, user $APOLLO_USER)"
docker exec -u "$APOLLO_USER" "$APOLLO_C" bash -lc "
  cd /apollo && bazel build $PKG:all &&
  mkdir -p /apollo/modules/calibration/fast_calib/bin &&
  cp -f bazel-bin/modules/calibration/fast_calib/{fast_calib,multi_fast_calib,lidar_center_test} \
        /apollo/modules/calibration/fast_calib/bin/
"

mkdir -p "$REPO/build"
cp -f "$PKG_HOST"/bin/{fast_calib,multi_fast_calib,lidar_center_test} "$REPO/build/"
echo "[apollo-build] done -> $REPO/build/{fast_calib,multi_fast_calib,lidar_center_test}"
