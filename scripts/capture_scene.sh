#!/usr/bin/env bash
# Capture one calibration scene for FAST-Calib2:
#   - grab one camera frame (RTSP)         -> calib_data/<cam>/<scene>/image.png
#   - record LiDAR clouds, fuse frames,    -> calib_data/<cam>/<scene>/cloud.bag
#     and wrap into a ROS bag (topic $TOPIC, ring synthesized for mech LiDARs)
#
# This is an EXAMPLE for an Apollo Cyber RT LiDAR source (records with
# cyber_recorder inside the Apollo container, then converts). For a native ROS
# LiDAR just `rosbag record` the topic instead; the FAST-Calib2 side is identical.
#
# Configure via env (no secrets are baked in):
#   RTSP=rtsp://user:pass@host:554/live   # camera stream (required)
#   APOLLO_C=<apollo_container_name>       # running Apollo container (required)
#   CH=/apollo/sensor/<lidar>/PointCloud2  # source LiDAR channel (required)
#   TOPIC=/lidar_points  FRAME=lidar       # output bag topic / frame_id
#   ROS_IMG=fast-calib2:noetic  RING_FLAG= # (--no-ring for solid pipeline)
#
# Usage:  capture_scene.sh <cam> <scene> [seconds]
set -euo pipefail

CAM="${1:?usage: capture_scene.sh <cam> <scene> [seconds]}"
SCENE="${2:?usage: capture_scene.sh <cam> <scene> [seconds]}"
SECS="${3:-5}"

RTSP="${RTSP:?set RTSP=rtsp://user:pass@host:554/live}"
APOLLO_C="${APOLLO_C:?set APOLLO_C=<running apollo container name>}"
CH="${CH:?set CH=/apollo/sensor/<lidar>/PointCloud2}"
TOPIC="${TOPIC:-/lidar_points}"
FRAME="${FRAME:-lidar}"
ROS_IMG="${ROS_IMG:-fast-calib2:noetic}"
RING_FLAG="${RING_FLAG:-}"                 # set --no-ring for the solid pipeline

# repo root = parent of this script's dir; helper scripts live beside this one
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$TOOLS/.." && pwd)"
# host dir that is bind-mounted into the Apollo container as /apollo/data
APOLLO_DATA_HOST="${APOLLO_DATA_HOST:?set APOLLO_DATA_HOST=<host path mounted as /apollo/data>}"

DEST="$REPO/calib_data/$CAM/$SCENE"
WORK="$APOLLO_DATA_HOST/calib_capture/$CAM/$SCENE"   # == /apollo/data/calib_capture/... in container
mkdir -p "$DEST"

echo "[capture] $CAM/$SCENE : recording lidar ${SECS}s + grabbing image"

# --- lidar record (background, inside apollo container; root creates WORK) ---
docker exec "$APOLLO_C" bash -lc "
  source /apollo/cyber/setup.bash >/dev/null 2>&1
  export CYBER_IP=127.0.0.1 CYBER_DOMAIN_ID=80
  mkdir -p /apollo/data/calib_capture/$CAM/$SCENE
  cd /apollo/data/calib_capture/$CAM/$SCENE
  rm -f rec.*
  timeout -s INT $SECS cyber_recorder record -c $CH -o rec >/dev/null 2>&1 || true
  chmod -R a+rwX /apollo/data/calib_capture/$CAM
" &
LIDAR_PID=$!

# --- image grab (10th decoded frame -> avoids any partial first frame) ---
ffmpeg -nostdin -loglevel error -rtsp_transport tcp -i "$RTSP" \
       -vsync 0 -frames:v 10 -update 1 -q:v 2 -y "$DEST/image.png"

wait "$LIDAR_PID" || true

# --- keep the raw cyber record: fast_calib reads it directly (no ROS bag) ---
mkdir -p "$DEST/record"
cp -f "$WORK"/rec.* "$DEST/record/"

# --- fuse frames -> PCD (host, cyber_record reader; feeds pick_roi.py / QA) ---
python3 "$TOOLS/record_to_pcd.py" \
  --record-glob "$WORK/rec.*" --channel "$CH" --out "$DEST/cloud.pcd" --max-frames 1000

echo "[capture] DONE -> $DEST"
ls -la "$DEST"
