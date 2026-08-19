# FAST-Calib2

## LiDAR-Camera Extrinsic Calibration with Reflective Annular Targets

FAST-Calib2 extends [FAST-Calib](https://github.com/hku-mars/FAST-Calib) to LiDAR-camera modules that were previously hard to calibrate due to **low-quality point clouds**. With a custom-designed reflective annular calibration target, it enables robust center extraction on **large-spot solid-state and mechanical LiDARs**, including Mid360, Avia, Ouster, XT32, JT128, Airy, E1R, and Adaps Photonics Spad LiDAR.

**Key highlights include:**

1. A self-designed 3D reflective annular calibration target that avoids center extraction errors caused by hole-edge inflation and bleeding artifacts in previous circular-hole calibration boards.
2. A robust concentric-circle fitting method that uses the fixed inner and outer annulus radii as geometric constraints.
3. Automatic calibration board ROI extraction without manual pass-through tuning.
4. Geometry and radius quality checks for extracted annulus centers.
5. Single-scene and multi-scene LiDAR-camera extrinsic calibration without initial extrinsic parameters.

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at zhengcr@connect.hku.hk.

<p align="center">
  <img src="./pics/cover.jpg" width="100%">
  <font color=#a0a0a0 size=2>Mid360 calibration example.</font>
</p>

## 📖 Documentation

- **[Simple guide](docs/simple_guide.md)** — shortest path: clone → build in the Apollo container → one-line single/multi-scene calibration → standalone LiDAR extraction test.
- **[Detailed LiDAR↔Camera calibration guide](docs/lidar2camera_calibration_guide.md)** ([中文](docs/lidar2camera_calibration_guide_zh.md)) — end-to-end: environment setup, data capture, formatting, calibration, and evaluation, including Apollo/Cyber sources, multi-scene, and troubleshooting.
- **[Capture & QA helper scripts](scripts/README_lidar2cam_capture.md)** — `capture_scene.sh`, `apollo_build.sh`, `record_to_pcd.py`, `pick_roi.py`, `overlay_reproj.py` / `render_scene_qa.py`, `multi_capture.sh` / `pick_multi_roi.sh`, `intrinsic_board_check.py`.
- **[What changed in the ROS-free refactor](docs/ros_free_changes.md)** — team-facing summary: Apollo-native input, build, migration notes, validation.

## 1. Prerequisites & Build

**No ROS required.** Point clouds are read natively from **Apollo Cyber RT
record files** (`cyber_recorder` output, channel with
`apollo.drivers.PointCloud` messages) or from PCD files.

**Primary build — inside the Apollo dev environment (bazel):** uses the Apollo
workspace's own third-party modules (`@local_config_pcl`, `@opencv`,
`@eigen`); needs a running `apollo_dev` container.

```bash
APOLLO_HOST=/path/to/apollo scripts/apollo_build.sh
# -> build/{fast_calib, multi_fast_calib, lidar_center_test}
```

**Fallback — plain CMake** (PCL>=1.8, OpenCV>=4.0, Eigen3, CMake>=3.10) for
machines without an Apollo checkout:

```bash
mkdir -p build && cd build && cmake .. && make -j
```

## 2. Calibration Target

FAST-Calib2 uses four reflective annuli and four visual markers on one board. The annuli are used by LiDAR center extraction, while the visual markers are used by the camera pipeline.

Materials:

- Board: PVC
- Reflective annulus stickers: 3M engineering-grade reflective film

<p align="center">
  <img src="./pics/FAST-Calib2-board.png" width="100%">
  <font color=#a0a0a0 size=2>Reflective annular calibration target and annotated dimensions.</font>
</p>

DIY Calibration Target Tips:

1. Fabricate the board based on the schematic. Ensure a minimum thickness of 1 cm to avoid bending.
2. Apply reflective annulus stickers to the designated ring positions on the fabricated board.

## 3. Method Overview

Both LiDAR pipelines first **locate the calibration board automatically**, fit the board plane, and align the plane to `Z=0`. Center extraction is then performed in the aligned board frame.

Solid-state LiDAR pipeline:

1. Extract high-reflectivity annulus points on the fitted board plane.
2. Cluster the extracted annulus points.
3. Fit robust single circles as the default center estimate.
4. Optionally extract annulus boundary points and fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

Mechanical LiDAR pipeline:

1. Use LiDAR `ring` order to find intensity transition points on the annulus boundary.
2. Try both interpolated boundary points and high-reflectivity-side boundary points.
3. Cluster the extracted boundary points.
4. Fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

The final quality checks include center-to-center geometry error and annulus radius consistency.

## 4. Quickstart: extrinsic calibration workflow

The steps below are the complete LiDAR↔camera workflow; the
[detailed guide](docs/lidar2camera_calibration_guide.md) expands each one.

**Step 1 — Build** (once; see §1 for both options):

```bash
APOLLO_HOST=/path/to/apollo scripts/apollo_build.sh   # Apollo env (bazel)
# or:  mkdir -p build && cd build && cmake .. && make -j
```

**Step 2 — Per-camera config.** Copy `config/cameras/_template.yaml` to
`config/cameras/<cam>.yaml` and fill in the camera intrinsics (calibrate this
exact camera first — a wrong focal length shifts the extrinsic translation),
the measured board geometry, and the LiDAR source:

```yaml
lidar_channel: "/apollo/sensor/<lidar>/PointCloud2"  # cyber record channel
lidar_frame: "<lidar_frame>"    # frame names used in the Apollo extrinsics YAML
camera_frame: "<cam>"
max_fusion_frames: 0            # 0 = fuse the whole record (scene is static)
beam_altitudes_deg: [...]       # ONLY for low-line mechanical LiDARs; omit for
                                # dense/solid (AT128, Livox) -> solid pipeline
```

**Step 3 — Capture a scene** (one board placement seen sharply by both
sensors). `scripts/capture_scene.sh` grabs an RTSP frame and records the
Apollo channel; or assemble the layout by hand:

```
calib_data/<cam>/<scene>/image.png       # camera frame
calib_data/<cam>/<scene>/record/rec.*    # cyber record file(s)  (or cloud.pcd)
```

**Step 4 — Board ROI.** Try `use_auto_lidar_roi: true` first. If the auto ROI
fails (sparse cloud, other retroreflectors, wall behind the board), pick a
tight manual box on the board:

```bash
python3 scripts/pick_roi.py calib_data/<cam>/<scene>/cloud.pcd --yaml config/cameras/<cam>.yaml
# also writes calib_data/<cam>/<scene>/cloud_roi.txt, auto-applied per scene
```

**Step 5 — Single-scene calibration** (repeat steps 3–5 for **≥3 board
poses**; each successful run appends its center pairs to
`output/<cam>/circle_center_record.txt`):

```bash
./build/fast_calib --config config/cameras/<cam>.yaml \
                   --scene calib_data/<cam>/<scene> --output output/<cam>
# a good run: camera "4 centers found", LiDAR 4 concentric annuli at the
# board radii, [Result] RMSE of a few mm; exit code 0
```

**Step 6 — Multi-scene joint fit** (uses the last 3 recorded scenes):

```bash
./build/multi_fast_calib --config config/cameras/<cam>.yaml --output output/<cam>
```

**Step 7 — Evaluate** before trusting the result: reproject the LiDAR into
every scene's image — the high-intensity (red) points must land on the 4 white
rings in **all** scenes:

```bash
python3 scripts/render_scene_qa.py calib_data/<cam>/<scene> \
        output/<cam>/multi_calib_result.txt --config config/cameras/<cam>.yaml \
        --overlay output/<cam>/reproj_<scene>.png
```

**Outputs** in `output/<cam>/`: `single_calib_result.txt` /
`multi_calib_result.txt` (FAST-LIVO2 format, `T_cam_lidar`) and
`single_calib_extrinsics.yaml` / `multi_calib_extrinsics.yaml` (Apollo
convention: LiDAR as parent frame, camera as child — the camera pose in the
LiDAR frame, drop-in for Apollo perception params). `--debug-dir <dir>` writes
every intermediate cloud as a PCD (replaces the former RViz topics).

One-command orchestration of steps 3–7: `scripts/multi_capture.sh` (auto-ROI)
or `scripts/pick_multi_roi.sh` (hand-picked ROI per scene).

Typical multi-scene target placement:

<p align="center">
  <img src="./pics/multi-scene.jpg" width="100%">
  <font color=#a0a0a0 size=2>Placement of the calibration target for multi-scene data collection: (a) facing forward, (b) oriented to the right, (c) oriented to the left.</font>
</p>

## 5. Standalone LiDAR Center Extraction Test

<details>
<summary>Show Unit Test Usage</summary>

The repository also provides a LiDAR-only test tool for checking annulus center extraction before running full camera-LiDAR calibration.

Run solid-state LiDAR data (PCD or cyber record input):

```bash
./build/lidar_center_test --config config/qr_params.yaml \
    calib_data/fast-calib2-data/left.pcd - solid
./build/lidar_center_test --config config/cameras/<cam>.yaml \
    calib_data/<cam>/<scene>/record /apollo/sensor/<lidar>/PointCloud2 solid
```

Run mechanical LiDAR data (set `beam_altitudes_deg` in the config to
synthesize the scan-ring index for clouds without a `ring` field):

```bash
./build/lidar_center_test --config config/cameras/<cam>.yaml \
    calib_data/<cam>/<scene>/cloud.pcd - mech
```

The test tool writes:

- `*_centers.txt`: extracted annulus center coordinates
- `*_debug_cloud.pcd`: board point cloud, annulus points, boundary points, and center markers for visualization

Debug PCD colors:

- Board points: intensity color map
- Annulus points: green
- Solid-LiDAR boundary points: red
- Centers: white spheres

</details>
