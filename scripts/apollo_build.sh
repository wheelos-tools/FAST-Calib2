#!/usr/bin/env bash
# Build FAST-Calib2 inside the Apollo dev environment via apollo.sh (no ROS).
#
# Syncs the sources into <apollo workspace>/modules/calibration/fast_calib,
# then runs `./apollo.sh <cmd> calibration/fast_calib` inside the running
# Apollo dev container — scoped to this one module, so it does NOT build the
# whole Apollo tree — and copies the binaries back to <repo>/build/.
#
# Configure via env:
#   APOLLO_HOST=/path/to/apollo      # host path of the Apollo workspace
#                                     (mounted as /apollo in the container)
#   APOLLO_C=<container>              # running Apollo dev container
#                                     (default: first apollo_dev_* found)
#   APOLLO_USER=nvidia                # user to build as inside the container
#   APOLLO_BUILD_CMD=build_opt        # apollo.sh subcommand (build, build_opt,
#                                     # build_cpu, ...)
#
# Usage:  scripts/apollo_build.sh
set -euo pipefail

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$TOOLS/.." && pwd)"

APOLLO_HOST="${APOLLO_HOST:?set APOLLO_HOST=<host path of the Apollo workspace>}"
APOLLO_C="${APOLLO_C:-$(docker ps --format '{{.Names}}' | grep -m1 apollo_dev || true)}"
[ -n "$APOLLO_C" ] || { echo "ERROR: no running apollo_dev container; set APOLLO_C" >&2; exit 1; }
APOLLO_USER="${APOLLO_USER:-nvidia}"

APOLLO_BUILD_CMD="${APOLLO_BUILD_CMD:-build_opt}"
MODULE="calibration/fast_calib"
PKG_HOST="$APOLLO_HOST/modules/$MODULE"

echo "[apollo-build] sync sources -> $PKG_HOST"
mkdir -p "$PKG_HOST"
rsync -a --delete \
  --include='/BUILD' --include='/src/***' --include='/include/***' \
  --include='/proto/***' --include='/config/***' \
  --exclude='*' \
  "$REPO/" "$PKG_HOST/"

echo "[apollo-build] ./apollo.sh $APOLLO_BUILD_CMD $MODULE  (container $APOLLO_C, user $APOLLO_USER)"
docker exec -u "$APOLLO_USER" "$APOLLO_C" bash -lc "
  cd /apollo && ./apollo.sh $APOLLO_BUILD_CMD $MODULE &&
  mkdir -p /apollo/modules/$MODULE/bin &&
  cp -f bazel-bin/modules/$MODULE/{fast_calib,multi_fast_calib,lidar_center_test} \
        /apollo/modules/$MODULE/bin/
"

mkdir -p "$REPO/build"
cp -f "$PKG_HOST"/bin/{fast_calib,multi_fast_calib,lidar_center_test} "$REPO/build/"
echo "[apollo-build] done -> $REPO/build/{fast_calib,multi_fast_calib,lidar_center_test}"
