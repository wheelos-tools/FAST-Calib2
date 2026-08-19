# LiDAR ↔ Camera Extrinsic Calibration Guide (FAST-Calib2)

Calibrate the rigid transform between a **camera** and a **LiDAR** with
FAST-Calib2's reflective-annular-target pipeline. Point clouds are read
natively from **Apollo Cyber records** (or PCD files) — no ROS anywhere.

**Result:** `T_cam_lidar` (`P_cam = R · P_lidar + t`, FAST-LIVO2 txt) plus an
Apollo-convention extrinsics YAML (camera pose in the LiDAR frame) that drops
into Apollo perception params.

**How it works:** one scene = one board placement captured by both sensors.
The camera detects 4 ArUco markers → board pose → 4 ring centers; the LiDAR
extracts the 4 reflective-annulus centers; SVD aligns the two sets. One scene
is sensitive; ≥3 scenes joined are robust.

## 1. Deploy the code

```bash
cd ~/workspace/01code                # or any directory you prefer
git clone https://github.com/wheelos-tools/FAST-Calib2.git
cd FAST-Calib2
git checkout feat/ros-free-apollo
```

## 2. Move the code into Apollo and compile

FAST-Calib2 builds as a normal Apollo module (scoped — it does not build the
Apollo tree):

```bash
APOLLO=/path/to/apollo    # your Apollo workspace

# move the code into the workspace
mkdir -p $APOLLO/modules/calibration
cp -r ~/workspace/01code/FAST-Calib2 $APOLLO/modules/calibration/fast_calib

# start and enter the Apollo dev container
cd $APOLLO
bash docker/scripts/dev_start.sh      # skip if apollo_dev_* is already running
bash docker/scripts/dev_into.sh

# standard Apollo build (inside the container), scoped to this module
bash apollo.sh build_opt calibration/fast_calib

# stay inside the container and run straight from bazel-bin — no copy step
cd /apollo/modules/calibration/fast_calib
FC=/apollo/bazel-bin/modules/calibration/fast_calib
```

First build compiles PCL/OpenCV once (slow); rebuilds are incremental.
**From here on, work inside the container from
`/apollo/modules/calibration/fast_calib`**, calling the binaries via
`$FC/...` (the `bazel-bin` symlink is guaranteed to resolve there).

> No Apollo checkout? Fallback:
> `sudo apt install cmake libpcl-dev libopencv-dev libeigen3-dev`, then
> `mkdir -p build && cd build && cmake .. && make -j` and use
> `FC=./build` in the commands below.

## 3. Run calibration

Per-scene data in `calib_data/<cam>/<scene>/`:

- `image.png` — the camera frame
- `record/` — cyber record file(s) of the LiDAR channel, **or** `cloud.pcd`
- optional `cloud_roi.txt` (from `scripts/pick_roi.py`) — auto-applied manual ROI

Per-camera config `config/cameras/<cam>.yaml` (copy `_template.yaml`):
intrinsics of **this exact camera** (wrong focal length shifts the extrinsic
translation), measured board geometry, `lidar_channel` +
`lidar_frame`/`camera_frame`, ROI. Only for low-line mechanical LiDARs add
`beam_altitudes_deg: [...]` (dense/solid LiDARs like AT128/Livox: omit).

Single-scene calibration (repeat for ≥3 board poses):

```bash
$FC/fast_calib --config config/cameras/<cam>.yaml --scene calib_data/<cam>/<scene> --output output/<cam>
```

A good run prints camera `4 centers found`, LiDAR
`Number of edge clusters: 4`, `[Result] RMSE: 0.00xx m` (a few mm), and exits 0.

Multi-scene joint calibration (uses the last ≥3 recorded scenes):

```bash
$FC/multi_fast_calib --config config/cameras/<cam>.yaml --output output/<cam>
```

Verify by reprojection before trusting the result — red high-intensity points
must land on the 4 white rings in **every** scene:

```bash
python3 scripts/render_scene_qa.py calib_data/<cam>/<scene> output/<cam>/multi_calib_result.txt --config config/cameras/<cam>.yaml --overlay output/<cam>/reproj_<scene>.png
```

Outputs in `output/<cam>/`: `single/multi_calib_result.txt` (`T_cam_lidar`)
and `single/multi_calib_extrinsics.yaml` (Apollo convention, perception
drop-in). `--debug-dir <dir>` writes every intermediate cloud as a PCD.

## 4. Standalone LiDAR Center Extraction Test

Check annulus extraction alone (no camera) before full calibration:

```bash
# solid / dense LiDAR (Livox, AT128, ...), PCD or cyber-record input:
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - solid
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/record /apollo/sensor/<lidar>/PointCloud2 solid

# mechanical LiDAR (needs beam_altitudes_deg in the config):
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - mech
```

Writes `*_centers.txt` (the 4 centers) and `*_debug_cloud.pcd` (board =
intensity colors, annulus = green, boundary = red, centers = white spheres).

## 5. Capturing data & tips

Capture one scene (RTSP frame + cyber record of the LiDAR channel):

```bash
RTSP=rtsp://user:pass@host:554/live APOLLO_C=<apollo_container> CH=/apollo/sensor/<lidar>/PointCloud2 scripts/capture_scene.sh <cam> <scene> 5
```

Or one command for the whole ≥3-scene flow: `scripts/multi_capture.sh <cam> 3 5`
(auto-ROI) / `scripts/pick_multi_roi.sh <cam>` (hand-picked ROI per scene).

- **Board placement:** both sensors see it sharply; as close as practical;
  >0.3 m from walls; moderate tilt; vary position between scenes.
- **Manual ROI** when auto fails (sparse cloud, other reflectors, wall behind):
  `python3 scripts/pick_roi.py calib_data/<cam>/<scene>/cloud.pcd --yaml config/cameras/<cam>.yaml`.
- **Hard clouds:** tune via config keys, no rebuild — e.g. sparse 16-line:
  `mech_cluster_min_size: 20`, `mech_cluster_tolerance: 0.13`; dense
  non-uniform (AT128): `annulus_cluster_min_size: 30`,
  `annulus_cluster_tolerance: 0.06–0.10`; bright ring points dropped
  "off-plane": widen `board/annulus_plane_inlier_threshold`.
- **Acceptance:** every scene has 4 markers + 4 annuli at the correct radii;
  RMSE ≤ ~1 cm; overlay red-on-rings in all scenes; `|t|` matches the physical
  mounting.
