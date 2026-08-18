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

- **[Detailed LiDAR↔Camera calibration guide](docs/lidar2camera_calibration_guide.md)** ([中文](docs/lidar2camera_calibration_guide_zh.md)) — end-to-end: environment setup, data capture, formatting, calibration, and evaluation, including Apollo/Cyber sources, multi-scene, and troubleshooting.
- **[Capture & QA helper scripts](scripts/README_lidar2cam_capture.md)** — `capture_scene.sh`, `record_to_pcd.py`, `pcd_to_bag.py`, `pick_roi.py`, `overlay_reproj.py` / `render_scene_qa.py`, `multi_capture.sh` / `pick_multi_roi.sh`, `intrinsic_board_check.py`.

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

## 4. Run Examples

Prepare static acquisition data per scene in
`calib_data/<cam>/<scene>/`:

- `image.png` — the camera frame
- `record/` — cyber record file(s) of the LiDAR channel, **or** `cloud.pcd`
- optional `cloud_roi.txt` (from `scripts/pick_roi.py`) — auto-applied manual ROI

Run single-scene calibration:

```bash
./build/fast_calib --config config/cameras/<cam>.yaml \
                   --scene calib_data/<cam>/<scene> --output output/<cam>
# containerized: docker/run.sh <cam> <scene>
```

After collecting at least three scenes, run multi-scene joint calibration:

```bash
./build/multi_fast_calib --config config/cameras/<cam>.yaml --output output/<cam>
# containerized: docker/run_multi.sh <cam>
```

Results: `single_calib_result.txt` / `multi_calib_result.txt` (FAST-LIVO2
format) plus `single_calib_extrinsics.yaml` / `multi_calib_extrinsics.yaml`
(Apollo transform format, `P_cam = R · P_lidar + t`). `--debug-dir <dir>`
writes every intermediate cloud as a PCD (replaces the former RViz topics).

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
