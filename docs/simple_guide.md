# FAST-Calib2 Simple Guide

The shortest path from a fresh machine to a LiDAR↔camera extrinsic, assuming
the calibration data (images + point clouds) is already captured. For capture,
tuning, and troubleshooting see the
[detailed guide](lidar2camera_calibration_guide.md).

## 1. Get the code

Clone anywhere you keep projects — the tool is self-contained and nothing
depends on its location (on our Jetsons the convention is a workspace dir like
`/home/nvidia/workspace/01code/`):

```bash
cd ~/workspace/01code            # or any directory you prefer
git clone https://github.com/wheelos-tools/FAST-Calib2.git
cd FAST-Calib2
git checkout feat/ros-free-apollo
```

## 2. Compile (CMake)

Install the dependencies once, then build. No ROS, no Apollo checkout needed:

```bash
sudo apt install -y cmake libpcl-dev libopencv-dev libeigen3-dev

mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ..
```

This produces three binaries in `build/`:

| Binary | Purpose |
|---|---|
| `build/fast_calib` | single-scene calibration (one board placement) |
| `build/multi_fast_calib` | joint fit over the last ≥3 single-scene runs |
| `build/lidar_center_test` | LiDAR-only annulus-extraction check (optional) |

## 3. Configure your camera

Each camera has one config file. Copy the template and edit it:

```bash
cp config/cameras/_template.yaml config/cameras/cam0.yaml
```

Fill in these blocks (everything else can stay at the defaults):

```yaml
# 1) Camera intrinsics — from a chessboard calibration of THIS exact camera.
#    Wrong intrinsics silently shift the extrinsic translation.
  fx: ...
  fy: ...
  cx: ...
  cy: ...
  k1: ...
  k2: ...
  p1: ...
  p2: ...

# 2) Board geometry — measure your physical board [meters].
  marker_size: 0.20            # ArUco marker side length
  delta_width_qr_center: 0.55  # half horizontal distance between marker centers
  delta_height_qr_center: 0.35 # half vertical distance between marker centers
  delta_width_circles: 0.5     # horizontal distance between ring centers
  delta_height_circles: 0.4    # vertical distance between ring centers
  circle_radius: 0.12          # ring centerline radius
  annulus_half_width: 0.025

# 3) LiDAR source — only needed when the cloud comes from a cyber record;
#    a cloud.pcd input ignores the channel.
  lidar_channel: "/apollo/sensor/<lidar>/PointCloud2"
  lidar_frame: "<lidar_frame>"   # names written into the Apollo extrinsics YAML
  camera_frame: "cam0"
  # beam_altitudes_deg: [...]    # ONLY for low-line mechanical LiDARs (16-line
                                 # etc.); leave out for AT128 / Livox

# 4) Board ROI — where the board sits in the LiDAR frame.
  use_auto_lidar_roi: true       # try auto first
  # If auto fails, set a manual box (or run scripts/pick_roi.py, which writes
  # it here for you):
  x_min: ...
  x_max: ...
  y_min: ...
  y_max: ...
  z_min: ...
  z_max: ...
```

## 4. Calibrate

**Data layout.** Put each board placement ("scene") in its own folder:

```
calib_data/cam0/scene1/image.png       # the camera frame
calib_data/cam0/scene1/record/rec.*    # cyber record of the LiDAR channel
                                       #   ...or cloud.pcd (x y z intensity)
calib_data/cam0/scene1/cloud_roi.txt   # optional per-scene manual ROI
calib_data/cam0/scene2/...             # ≥3 scenes total, different board poses
calib_data/cam0/scene3/...
```

**Run each scene** (repeat for scene1..scene3):

```bash
./build/fast_calib --config config/cameras/cam0.yaml \
                   --scene calib_data/cam0/scene1 \
                   --output output/cam0
```

A good run prints: camera `4 centers found`, LiDAR `Number of edge clusters: 4`
with concentric fits at your board's radii, and `[Result] RMSE: 0.00xx m`
(a few millimeters), then exits with code 0. Each success appends the extracted
centers to `output/cam0/circle_center_record.txt`.

If the LiDAR side finds fewer than 4 clusters: pick a tighter board ROI
(`scripts/pick_roi.py calib_data/cam0/scene1/cloud.pcd --yaml config/cameras/cam0.yaml`)
and rerun that scene.

**Joint fit** (after ≥3 successful scenes):

```bash
./build/multi_fast_calib --config config/cameras/cam0.yaml --output output/cam0
```

**Check the result** — reproject the LiDAR into each scene's image; the
high-intensity (red) points must land on the 4 white rings in every scene:

```bash
python3 -m pip install --user open3d opencv-python numpy   # once
python3 scripts/render_scene_qa.py calib_data/cam0/scene1 \
        output/cam0/multi_calib_result.txt --config config/cameras/cam0.yaml \
        --overlay output/cam0/reproj_scene1.png
```

**Use the result** — in `output/cam0/`:

- `multi_calib_result.txt` — `T_cam_lidar` (`P_cam = R·P_lidar + t`),
  FAST-LIVO2 format.
- `multi_calib_extrinsics.yaml` — the same transform in Apollo's extrinsics
  convention (LiDAR as parent frame, camera as child = camera pose in the
  LiDAR frame); drop it into the Apollo perception params directory.
