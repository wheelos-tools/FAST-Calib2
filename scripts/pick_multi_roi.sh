#!/usr/bin/env bash
# Hand-pick the board ROI for each already-captured scene, auto-update the config
# yaml (use_auto_lidar_roi: false + picked ROI), run that scene's single-scene
# calibration, then combine the scenes into a joint extrinsic.
#
#   pick_multi_roi.sh [cam] [scene1 scene2 ...]
#   e.g.  pick_multi_roi.sh cam0
#
# The Open3D picker opens on the Orin monitor (DISPLAY=:1). Data must already be
# captured (image.png + cloud.pcd per scene).
set -uo pipefail

CAM="${1:-cam0}"; shift 2>/dev/null || true
SCENES=("$@"); [ ${#SCENES[@]} -eq 0 ] && SCENES=(scene1 scene2 scene3)

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # this scripts/ dir
REPO="$(cd "$TOOLS/.." && pwd)"                          # FAST-Calib2 root
IMG="${ROS_IMG:-fast-calib2:noetic}"
CFG="$REPO/config/cameras/$CAM.yaml"
REC="$REPO/output/$CAM/circle_center_record.txt"
PY="${PYTHON:-python3}"                     # must have open3d + opencv (not conda base)
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

[ -f "$CFG" ] || { echo "ERROR: config not found: $CFG" >&2; exit 1; }
mkdir -p "$REPO/output/$CAM"
# clear the center record via docker (it is root-owned; the calib runs recreate it)
docker run --rm -v "$REPO:/w" "$IMG" bash -lc "rm -f /w/output/$CAM/circle_center_record.txt" >/dev/null 2>&1 || true

CUR=""
cleanup(){ [ -n "$CUR" ] && docker kill "$CUR" >/dev/null 2>&1; CUR=""; return 0; }
trap cleanup EXIT
trap 'cleanup; echo; echo "[pick] interrupted — exiting."; exit 130' INT TERM

run_scene() {
  local s="$1"
  local cn="cal_${s}_$$"
  local log="/tmp/${cn}.log"
  CUR="$cn"
  docker run --rm --name "$cn" --net=host \
    -v "$REPO:/w" \
    "$IMG" /w/docker/.ws_build/fast_calib \
      --config "/w/config/cameras/$CAM.yaml" \
      --scene "/w/calib_data/$CAM/$s" \
      --output "/w/output/$CAM" >"$log" 2>&1
  CUR=""
  if grep -qa "Saved four pairs of target centers" "$log"; then
    echo "  $s OK   $(grep -a 'Result] RMSE' "$log" | tail -1 | sed 's/\x1b\[[0-9;]*m//g')"
    return 0
  fi
  echo "  $s FAILED (no 4 centers) — check the ROI / angle. log: $log"
  return 1
}

for s in "${SCENES[@]}"; do
  echo
  echo "==================== $s ===================="
  pcd="$REPO/calib_data/$CAM/$s/cloud.pcd"
  [ -f "$pcd" ] || { echo "  no cloud.pcd for $s — skipping"; continue; }
  while true; do
    read -r -p ">>> $s: press Enter to open the ROI picker (s=skip)... " ans
    [ "$ans" = "s" ] && { echo "  skipped $s"; break; }
    if ! "$PY" "$TOOLS/pick_roi.py" "$pcd" --yaml "$CFG"; then
      echo "  pick cancelled (<4 points)."; read -r -p "  retry? [Y/n] " r; [ "$r" = "n" ] && break; continue
    fi
    echo "[run] calibrating $s ..."
    if run_scene "$s"; then break; fi
    read -r -p "  re-pick this scene (tighter ROI on the board face)? [Y/n] " r
    [ "$r" = "n" ] && break
  done
done

blocks=$(grep -ca "lidar_centers:" "$REC" 2>/dev/null || echo 0)
echo
echo "[multi] good scenes recorded: $blocks"
if [ "$blocks" -lt 3 ]; then
  echo "ERROR: need >=3 successful scenes for the joint fit (have $blocks)." >&2
  echo "       Re-pick the failed scenes, or run the joint step later: docker/run_multi.sh $CAM" >&2
  exit 2
fi

echo "[multi] joint fit ..."
docker run --rm --net=host \
  -v "$REPO:/w" \
  "$IMG" /w/docker/.ws_build/multi_fast_calib \
    --config "/w/config/cameras/$CAM.yaml" \
    --output "/w/output/$CAM" >/tmp/${CAM}_multi.log 2>&1 || true

RESULT="$REPO/output/$CAM/multi_calib_result.txt"
echo
echo "======================= JOINT RESULT ======================="
cat "$RESULT" 2>/dev/null || { echo "(not produced — see /tmp/${CAM}_multi.log)"; exit 3; }
echo "============================================================"

# per-scene QA from the JOINT extrinsic: overlay PNG + colored PCD for every scene
# (the joint result has no intrinsics, so render_scene_qa reads them from the config)
echo "[multi] rendering per-scene QA (overlay + colored pcd) ..."
for s in "${SCENES[@]}"; do
  [ -f "$REPO/calib_data/$CAM/$s/image.png" ] || continue
  "$PY" "$TOOLS/render_scene_qa.py" "$REPO/calib_data/$CAM/$s" "$RESULT" \
    --config "$CFG" \
    --overlay "$REPO/output/$CAM/reproj_$s.png" \
    --colored "$REPO/output/$CAM/colored_$s.pcd" 2>/dev/null \
    && echo "  $s -> reproj_$s.png + colored_$s.pcd"
done
echo "[multi] DONE. Outputs in $REPO/output/$CAM/"
