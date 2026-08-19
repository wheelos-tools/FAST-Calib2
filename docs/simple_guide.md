# FAST-Calib2 Simple Guide

Minimal end-to-end usage, mirroring the official run examples: deploy, build
in the Apollo environment, calibrate with one command per step. Assumes the
calibration data is already captured (see the
[detailed guide](lidar2camera_calibration_guide.md) for capture, config
details, and troubleshooting).

## 1. Deploy the code

```bash
cd ~/workspace/01code                # or any directory you prefer
git clone https://github.com/wheelos-tools/FAST-Calib2.git
cd FAST-Calib2
git checkout feat/ros-free-apollo
```

## 2. Start the Apollo container and compile

The build runs inside the Apollo dev environment (bazel, scoped to this module
only — it does not build the Apollo tree). Start the dev container from your
Apollo workspace if one is not already running, then compile:

```bash
# start the Apollo dev container (skip if apollo_dev_* is already running)
cd /path/to/apollo && bash docker/scripts/dev_start.sh

# compile FAST-Calib2 (from the FAST-Calib2 checkout)
cd ~/workspace/01code/FAST-Calib2
APOLLO_HOST=/path/to/apollo scripts/apollo_build.sh
```

Output: `build/{fast_calib, multi_fast_calib, lidar_center_test}` — statically
linked against the workspace's PCL/OpenCV, so they run directly on the host.
The first build compiles PCL/OpenCV once (slow); rebuilds are incremental.

> No Apollo checkout on the machine? Fallback:
> `sudo apt install cmake libpcl-dev libopencv-dev libeigen3-dev` then
> `mkdir -p build && cd build && cmake .. && make -j`.

## 3. Run calibration

Prepare static acquisition data per scene in `calib_data/<cam>/<scene>/`:

- `image.png` — the camera frame
- `record/` — cyber record file(s) of the LiDAR channel, **or** `cloud.pcd`
- optional `cloud_roi.txt` (from `scripts/pick_roi.py`) — auto-applied manual ROI

and a per-camera config `config/cameras/<cam>.yaml` (copy
`_template.yaml`: intrinsics, board geometry, `lidar_channel`, ROI).

Run single-scene calibration:

```bash
./build/fast_calib --config config/cameras/<cam>.yaml --scene calib_data/<cam>/<scene> --output output/<cam>
```

After collecting at least three scenes (run the command above once per scene),
run multi-scene joint calibration:

```bash
./build/multi_fast_calib --config config/cameras/<cam>.yaml --output output/<cam>
```

Results in `output/<cam>/`: `single_calib_result.txt` /
`multi_calib_result.txt` (FAST-LIVO2 format, `T_cam_lidar`) and
`single_calib_extrinsics.yaml` / `multi_calib_extrinsics.yaml` (Apollo
convention — camera pose in the LiDAR frame, drop-in for Apollo perception
params). A good run prints camera `4 centers found`, LiDAR
`Number of edge clusters: 4`, and `[Result] RMSE: 0.00xx m` (a few mm).

Verify by reprojection before trusting the result:

```bash
python3 scripts/render_scene_qa.py calib_data/<cam>/<scene> output/<cam>/multi_calib_result.txt --config config/cameras/<cam>.yaml --overlay output/<cam>/reproj_<scene>.png
```

## 4. Standalone LiDAR Center Extraction Test

Check annulus center extraction alone (no camera) before running full
camera-LiDAR calibration.

Run solid-state / dense LiDAR data (Livox, AT128, ...):

```bash
./build/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - solid
./build/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/record /apollo/sensor/<lidar>/PointCloud2 solid
```

Run mechanical LiDAR data (set `beam_altitudes_deg` in the config so the
scan-ring index is synthesized for clouds without a `ring` field):

```bash
./build/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - mech
```

The test tool writes into the config's output directory:

- `*_centers.txt` — extracted annulus center coordinates
- `*_debug_cloud.pcd` — board cloud, annulus points, boundary points, and
  center markers for visualization

Debug PCD colors: board points — intensity color map; annulus points — green;
solid-LiDAR boundary points — red; centers — white spheres.
