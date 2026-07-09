#!/usr/bin/env bash
# multi_capture.sh — collect several board angles and produce a JOINT LiDAR->camera
# extrinsic with FAST-Calib2. Creates a dated result id so nothing is overwritten and
# there is no root-owned record to clear.
#
#   multi_capture.sh [cam_base] [num_angles] [seconds]
#   e.g.  multi_capture.sh cam0 3 5
#
# For each angle it: pauses for you to reposition the board -> captures (image + fused
# cloud) -> runs single-scene calibration (auto-exits once the 4 centers are saved).
# After >=3 good angles it runs the joint multi-scene fit and an overlay for QA.
#
# It auto-starts the LiDAR driver (via cyber_launch, as the nvidia user) inside the Apollo
# container if the point-cloud channel isn't already live. LiDAR source defaults to the Hesai
# AT128; override via env (CH / TOPIC / FRAME / RING_FLAG / LIDAR_LAUNCH / LIDAR_USER) for a
# different sensor, e.g. LIDAR_LAUNCH=/apollo/modules/drivers/lidar/vanjeelidar/launch/vanjeelidar.launch
set -uo pipefail

BASE="${1:-cam0}"
N="${2:-3}"
SECS="${3:-5}"

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # this scripts/ dir
REPO="$(cd "$TOOLS/.." && pwd)"                          # FAST-Calib2 root
IMG="${ROS_IMG:-fast-calib2:noetic}"
PY="${PYTHON:-python3}"                     # must have open3d + opencv for QA rendering

# capture source (Hesai AT128 defaults; export before running to use another lidar)
export CH="${CH:-/apollo/sensor/hesai/main_front/PointCloud2}"
export TOPIC="${TOPIC:-/hesai_points}"
export FRAME="${FRAME:-hesai_main_front}"
export RING_FLAG="${RING_FLAG:---no-ring}"
APOLLO_C="${APOLLO_C:-$(docker ps --format '{{.Names}}' | grep -m1 apollo_dev)}"
export APOLLO_C
# LiDAR driver to auto-start if the channel isn't already live (match the source above).
# Start via cyber_launch AS THE SAME USER as the rest of the Apollo stack (nvidia) — running
# mainboard as root creates root-owned shm the nvidia stack can't write ("acquire block failed").
LIDAR_LAUNCH="${LIDAR_LAUNCH:-/apollo/modules/drivers/lidar/hesai/launch/hesai.launch}"
LIDAR_USER="${LIDAR_USER:-nvidia}"

DATE=$(date +%Y%m%d)
CAM="${BASE}_${DATE}"
CFG_SRC="$REPO/config/cameras/${BASE}.yaml"
CFG_DST="$REPO/config/cameras/${CAM}.yaml"
RECORD="$REPO/output/$CAM/circle_center_record.txt"

# ---------- preflight ----------
[ -f "$CFG_SRC" ] || { echo "ERROR: base config not found: $CFG_SRC" >&2; exit 1; }
[ -n "$APOLLO_C" ] || { echo "ERROR: no running apollo container found (set APOLLO_C)." >&2; exit 1; }

lidar_live() {
  # 'cyber_channel list' is flaky for discovery; subscribe directly and check for data.
  docker exec -u "$LIDAR_USER" "$APOLLO_C" bash -lc "source /apollo/cyber/setup.bash >/dev/null 2>&1; \
    export CYBER_IP=127.0.0.1 CYBER_DOMAIN_ID=80; timeout 6 cyber_channel echo $CH 2>/dev/null | head -c 300" \
    | grep -qa .
}
start_lidar() {
  echo "[multi] starting LiDAR driver in $APOLLO_C as $LIDAR_USER: cyber_launch $LIDAR_LAUNCH"
  docker exec -d -u "$LIDAR_USER" "$APOLLO_C" bash -lc "
    source /apollo/cyber/setup.bash >/dev/null 2>&1
    export CYBER_IP=127.0.0.1 CYBER_DOMAIN_ID=80 CYBER_PATH=/apollo/cyber
    cd /apollo
    nohup cyber_launch start $LIDAR_LAUNCH > /tmp/lidar_calib_launch.log 2>&1 &"
}

if lidar_live; then
  echo "[multi] LiDAR channel already live: $CH"
else
  start_lidar
  echo -n "[multi] waiting for DIFOP/first frame "
  for k in $(seq 1 15); do sleep 2; echo -n "."; lidar_live && break; done
  echo
  if lidar_live; then
    echo "[multi] LiDAR channel is live: $CH"
  else
    echo "WARNING: channel '$CH' still not live — check cabling / $LIDAR_LAUNCH / conf." >&2
    echo "         log: docker exec $APOLLO_C tail /tmp/lidar_calib_launch.log" >&2
    read -r -p "Continue anyway? [y/N] " a; [ "$a" = "y" ] || exit 1
  fi
fi

echo "[multi] result id : $CAM"
echo "[multi] angles    : $N   (>=3 needed for the joint fit)"
echo "[multi] lidar      : $CH  (ring: ${RING_FLAG:-on})"
cp "$CFG_SRC" "$CFG_DST"
mkdir -p "$REPO/output/$CAM"
for i in $(seq 1 "$N"); do mkdir -p "$REPO/calib_data/$CAM/scene$i"; done

CUR_CNAME=""
cleanup() { [ -n "$CUR_CNAME" ] && docker kill "$CUR_CNAME" >/dev/null 2>&1; CUR_CNAME=""; return 0; }
trap cleanup EXIT
trap 'cleanup; echo; echo "[multi] interrupted — exiting."; exit 130' INT TERM

# run one single-scene calibration; returns 0 if 4 centers were saved, 1 otherwise.
# auto-stops the container as soon as the result is written (no Ctrl-C, no RViz loop).
run_scene() {
  local cam="$1" scene="$2"
  local cname="fastcalib_${cam}_${scene}_$$"
  local log="/tmp/${cname}.log"
  CUR_CNAME="$cname"
  docker run --rm --name "$cname" --net=host \
    -v "$REPO:/root/calib_ws/src/fast_calib" \
    -v "$REPO/docker/.ws_build:/root/calib_ws/build" \
    -v "$REPO/docker/.ws_devel:/root/calib_ws/devel" \
    "$IMG" bash -lc "roslaunch fast_calib calib_cam.launch cam:=$cam scene:=$scene rviz:=false" \
    >"$log" 2>&1 &
  local pid=$! rc=2 k
  for k in $(seq 1 90); do
    if grep -qa "Saved four pairs of target centers" "$log"; then rc=0; break; fi
    if grep -qa "Need 4 centers, got" "$log";           then rc=1; break; fi
    kill -0 "$pid" 2>/dev/null || { rc=1; break; }
    sleep 2
  done
  docker kill "$cname" >/dev/null 2>&1; wait "$pid" 2>/dev/null; CUR_CNAME=""
  [ "$rc" = 0 ] && sed -n 's/\x1b\[[0-9;]*m//gp' "$log" | grep -a "Result] RMSE" | tail -1
  return "$rc"
}

# ---------- per-angle capture loop (with retry) ----------
good=0
for i in $(seq 1 "$N"); do
  while true; do
    echo
    echo "==================== ANGLE $i / $N ===================="
    read -r -p ">>> Position the board at angle $i, then press Enter to capture (s=skip): " ans
    [ "$ans" = "s" ] && { echo "  skipped angle $i"; break; }
    echo "[multi] capturing scene$i ..."
    if ! "$TOOLS/capture_scene.sh" "$CAM" "scene$i" "$SECS"; then
      echo "  capture failed."; read -r -p "  retry? [Y/n] " r; [ "$r" = "n" ] && break; continue
    fi
    echo "[multi] calibrating scene$i ..."
    if run_scene "$CAM" "scene$i"; then
      echo "  angle $i: OK"; good=$((good+1)); break
    else
      echo "  angle $i: FAILED to extract 4 LiDAR centers (board too oblique / out of view)."
      read -r -p "  retry this angle? [Y/n] " r; [ "$r" = "n" ] && break
    fi
  done
done

# ---------- joint fit ----------
blocks=$(grep -ca "lidar_centers:" "$RECORD" 2>/dev/null || echo 0)
echo
echo "[multi] good angles recorded: $blocks"
if [ "$blocks" -lt 3 ]; then
  echo "ERROR: need >=3 successful angles for the joint fit (have $blocks)." >&2
  echo "       Re-run to add more angles, then run the joint step manually:" >&2
  echo "       docker/run_multi.sh $CAM" >&2
  exit 2
fi

echo "[multi] running joint multi-scene calibration ..."
timeout 90 docker run --rm --net=host \
  -v "$REPO:/root/calib_ws/src/fast_calib" \
  -v "$REPO/docker/.ws_build:/root/calib_ws/build" \
  -v "$REPO/docker/.ws_devel:/root/calib_ws/devel" \
  "$IMG" bash -lc "roslaunch fast_calib multi_calib_cam.launch cam:=$CAM" >/tmp/${CAM}_multi.log 2>&1

RESULT="$REPO/output/$CAM/multi_calib_result.txt"
if [ ! -s "$RESULT" ]; then
  echo "ERROR: multi_calib_result.txt not produced — see /tmp/${CAM}_multi.log" >&2; exit 3
fi

echo
echo "======================= JOINT RESULT ======================="
cat "$RESULT"
echo "============================================================"

# ---------- per-scene QA from the JOINT extrinsic (overlay + colored PCD) ----------
# The joint result carries no intrinsics, so render_scene_qa reads them from the config.
echo "[multi] rendering per-scene QA (overlay + colored pcd) ..."
for i in $(seq 1 "$N"); do
  s="scene$i"
  [ -f "$REPO/calib_data/$CAM/$s/image.png" ] || continue
  "$PY" "$TOOLS/render_scene_qa.py" "$REPO/calib_data/$CAM/$s" "$RESULT" \
    --config "$REPO/config/cameras/$CAM.yaml" \
    --overlay "$REPO/output/$CAM/reproj_$s.png" \
    --colored "$REPO/output/$CAM/colored_$s.pcd" 2>/dev/null \
    && echo "  $s -> reproj_$s.png + colored_$s.pcd"
done

echo "[multi] DONE. All outputs in: $REPO/output/$CAM/"
