# LiDAR–Camera capture & QA tools

Helper scripts that feed FAST-Calib2 end to end: capture a scene, fuse a dense
point cloud, pick the board ROI, and visually verify the extrinsic. They are
source/LiDAR-agnostic and parameterized (no secrets or hardcoded hosts).

| Script | Purpose | Key options |
|---|---|---|
| `capture_scene.sh` | One scene: grab a camera frame + record/fuse the LiDAR → `calib_data/<cam>/<scene>/{image.png,cloud.pcd,cloud.bag}`. Example wiring is for an **Apollo Cyber RT** source; for a native ROS LiDAR just `rosbag record` the topic instead. | env: `RTSP`, `APOLLO_C`, `CH`, `TOPIC`, `FRAME`, `APOLLO_DATA_HOST` |
| `record_to_pcd.py` | Read source clouds and **fuse N static frames** into one dense ASCII PCD (`x y z intensity`). Uses the pure-python `cyber_record` reader. | `--record-glob`, `--channel`, `--out`, `--max-frames` |
| `pcd_to_bag.py` | PCD → single-message ROS bag. **Synthesizes a `ring` field** from beam elevation so mechanical LiDARs use FAST-Calib2's mechanical pipeline. | `--pcd`, `--bag`, `--topic`, `--frame`, `--no-ring` |
| `pick_roi.py` | Open3D interactive board-ROI picker (intensity-shaded). Shift-click ≥4 board points → prints the `x_min..z_max` block for the per-camera YAML. | `<cloud.pcd> [out.txt]` |
| `overlay_reproj.py` | Project the LiDAR into the camera image via the extrinsic, colored by reflectivity, for visual QA (red = reflective annuli land on the white rings; green crosshairs = detected centers). | `<scene_dir> <single_calib_result.txt> <out.png>` |

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

## Dependencies

- `record_to_pcd.py`: `pip install --user cyber_record protobuf==3.19.4` (Apollo source only).
- `pcd_to_bag.py`: a ROS environment (`rosbag`, `sensor_msgs`).
- `pick_roi.py`: `open3d` + a display.
- `overlay_reproj.py`: `opencv-python`, `numpy`.

## Notes

- **Frame fusion** densifies each scan-line azimuthally; it cannot add scan-lines (fixed by
  beam count), so also place the board close for low-line LiDARs.
- `pcd_to_bag.py`'s `ring` synthesis assumes nominal beam elevations — set them to your
  LiDAR's beam table (default is a 16-line unit spanning −16°…+14° at 2°).
- Very low line counts may need FAST-Calib2's mechanical cluster thresholds
  (`src/lidar_detect.hpp`) loosened; see the calibration guide.
