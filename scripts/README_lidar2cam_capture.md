# LiDAR–Camera capture & QA tools

Helper scripts that feed FAST-Calib2 end to end: capture a scene, fuse a dense
point cloud, pick the board ROI, and visually verify the extrinsic. They are
source/LiDAR-agnostic and parameterized (no secrets or hardcoded hosts).

| Script | Purpose | Key options |
|---|---|---|
| `capture_scene.sh` | One scene: grab a camera frame + record/fuse the LiDAR → `calib_data/<cam>/<scene>/{image.png,cloud.pcd,cloud.bag}`. Example wiring is for an **Apollo Cyber RT** source; for a native ROS LiDAR just `rosbag record` the topic instead. | env: `RTSP`, `APOLLO_C`, `CH`, `TOPIC`, `FRAME`, `APOLLO_DATA_HOST` |
| `record_to_pcd.py` | Read source clouds and **fuse N static frames** into one dense ASCII PCD (`x y z intensity`). Uses the pure-python `cyber_record` reader. | `--record-glob`, `--channel`, `--out`, `--max-frames` |
| `pcd_to_bag.py` | PCD → single-message ROS bag. **Synthesizes a `ring` field** from beam elevation so mechanical LiDARs use FAST-Calib2's mechanical pipeline. | `--pcd`, `--bag`, `--topic`, `--frame`, `--no-ring` |
| `pick_roi.py` | Open3D interactive board-ROI picker (intensity-shaded). Shift-click ≥4 board points → prints the `x_min..z_max` block; with `--yaml` writes it straight into the camera config (and sets `use_auto_lidar_roi: false`). | `<cloud.pcd> [--yaml cfg] [--pad m] [--out txt]` |
| `overlay_reproj.py` | Project the LiDAR into the camera image via a **single-scene** result, colored by reflectivity, for visual QA. | `<scene_dir> <single_calib_result.txt> <out.png>` |
| `render_scene_qa.py` | Per-scene QA from **any** result (single or **multi**): writes an overlay PNG + a colored PCD. Reads intrinsics from `--config` when the result file lacks them (the multi result has no intrinsics). | `<scene_dir> <result.txt> --config cfg --overlay png --colored pcd` |
| **`multi_capture.sh`** | Full **multi-scene** run: auto-starts the LiDAR driver, then per angle pauses to reposition the board → captures → single-scene calib → joint fit → per-scene QA. Uses auto-ROI. | `[cam_base] [num_angles] [sec]`; env `CH/TOPIC/FRAME/RING_FLAG/LIDAR_LAUNCH/LIDAR_USER` |
| **`pick_multi_roi.sh`** | Multi-scene with **hand-picked ROI** per scene (for tilted boards where auto-ROI fails): pick → auto-update yaml → calib → joint fit → per-scene QA. Data must already be captured. | `[cam] [scenes…]`; env `PYTHON`, `DISPLAY` |
| `intrinsic_board_check.py` | Validate camera intrinsics vs the physical board: independently reconstruct the 4 markers (per-marker solvePnP) and back-project the detected ring centers → distances vs ground truth. | `<image> fx fy cx cy k1 k2 p1 p2` |

## Quick flow

```bash
# 1. capture one scene (env: point at your camera + LiDAR source)
RTSP=rtsp://user:pass@host:554/live APOLLO_C=<apollo_container> \
CH=/apollo/sensor/<lidar>/PointCloud2 TOPIC=/lidar_points FRAME=lidar \
APOLLO_DATA_HOST=/path/mounted/as/apollo/data \
  scripts/capture_scene.sh cam0 scene1 5

# 2. pick the board ROI -> paste into config/cameras/cam0.yaml (use_auto_lidar_roi: false)
python3 scripts/pick_roi.py calib_data/cam0/scene1/cloud.pcd

# 3. run FAST-Calib2
roslaunch fast_calib calib_cam.launch cam:=cam0 scene:=scene1 rviz:=false

# 4. visual check
python3 scripts/overlay_reproj.py calib_data/cam0/scene1 \
  output/cam0/single_calib_result.txt output/cam0/reproj_overlay.png
```

## Multi-scene (joint) flow

Combining ≥3 board poses gives a far more robust extrinsic than a single scene.

```bash
# A. one command: capture 3 angles + joint fit + per-scene QA (auto-ROI)
#    (env points at your LiDAR; defaults shown are an Apollo/Hesai example)
scripts/multi_capture.sh cam0 3 5
#    -> config cam0_<date>.yaml, output/cam0_<date>/{multi_calib_result.txt,
#       reproj_scene{1,2,3}.png, colored_scene{1,2,3}.pcd}

# B. tilted boards where auto-ROI fails: hand-pick the ROI per scene
#    (data already captured under calib_data/<cam>/scene{1,2,3}/)
scripts/pick_multi_roi.sh cam0_<date>
#    per scene: Shift-click ≥4 board points -> Q; the yaml is updated automatically,
#    the scene is calibrated, then the joint fit + per-scene QA run.
```

Each single-scene run appends its 4 center pairs to `output/<cam>/circle_center_record.txt`;
the joint step (`multi_calib.launch`) reads the last ≥3 blocks. Per-scene QA uses the **joint**
extrinsic via `render_scene_qa.py`, so you get one overlay + one colored PCD per scene.

## Dependencies

- `record_to_pcd.py`: `pip install --user cyber_record protobuf==3.19.4` (Apollo source only).
- `pcd_to_bag.py`: a ROS environment (`rosbag`, `sensor_msgs`).
- `pick_roi.py`: `open3d` + a display.
- `overlay_reproj.py` / `render_scene_qa.py`: `opencv-python`, `numpy`.
- The `.sh` orchestrators expect `python3` (override with `PYTHON=`) to have `open3d`+`opencv`;
  on machines with conda active, point `PYTHON=/usr/bin/python3` at the system interpreter.

## Notes

- **Frame fusion** densifies each scan-line azimuthally; it cannot add scan-lines (fixed by
  beam count), so also place the board close for low-line LiDARs.
- `pcd_to_bag.py`'s `ring` synthesis assumes nominal beam elevations — set them to your
  LiDAR's beam table (default is a 16-line unit spanning −16°…+14° at 2°).
- Very low line counts may need FAST-Calib2's mechanical cluster thresholds
  (`src/lidar_detect.hpp`) loosened; see the calibration guide.
