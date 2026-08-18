# Refactor Plan: ROS-free FAST-Calib2 on Apollo Data

Goal: remove every ROS dependency (roscpp, catkin, rosbag, rosparam, RViz, the
`fast-calib2:noetic` container) and make **Apollo Cyber RT data the native
input**: point clouds come straight from `cyber_recorder` `.record` files
(`apollo.drivers.PointCloud`), config from plain YAML, results additionally in
Apollo extrinsics YAML format. The detection/solve algorithms are untouched.

---

## 1. Current state — where ROS actually lives

The calibration math (annulus extraction, concentric fitting, ArUco board pose,
SVD solve) is pure **PCL + OpenCV + Eigen**. ROS is only plumbing:

| Touchpoint | Files | What it does today |
|---|---|---|
| Input container | `src/data_preprocess.hpp`, `src/lidar_center_test.cpp` | `rosbag::View` over `sensor_msgs/PointCloud2` or Livox `CustomMsg` |
| Config | `include/common_lib.h` `loadParameters(nh)` + `launch/*.launch` + `config/cameras/*.yaml` | `rosparam` load + `nh.param` defaults, roslaunch arg substitution |
| Logging | all sources | `ROS_INFO/WARN/ERROR` (~40 call sites) |
| Debug viz | `src/lidar_detect.hpp` (8 publishers), `src/qr_detect.hpp` (1), `src/main.cpp` (2 + infinite `ros::Rate` spin loop for RViz) | publishes intermediate clouds; the spin loop is also the source of the known post-result SIGABRT on the car |
| Build | `CMakeLists.txt` (catkin), `package.xml`, `docker/` (ROS Noetic image) | catkin_make inside `fast-calib2:noetic` |
| Dead includes | `qr_detect.hpp` (cv_bridge, image_geometry, message_filters), `common_lib.h` (`tf/tf.h`, `pcl_ros`) | included but **unused** — delete outright |

Crucially, on the actual rigs (car-ningde-orin, wheel101) the data path is
already Apollo-first and the bag is a shim:

```
cyber_recorder .record ──record_to_pcd.py──▶ cloud.pcd ──pcd_to_bag.py (ROS container!)──▶ cloud.bag ──▶ C++
```

`pcd_to_bag.py` exists **only** because the binary demands a rosbag. Removing
ROS collapses the chain to `record → C++`.

---

## 2. Target architecture

```
calib_data/<cam>/<scene>/
    image.png            (ffmpeg RTSP grab, unchanged)
    record/rec.00000     (raw cyber_recorder output, copied as-is)
    cloud.pcd            (optional: cache emitted by the loader, feeds pick_roi/QA)

           ┌─────────────────────────────────────────────────────┐
           │ fast_calib (plain C++17 binary, no ROS)             │
           │                                                     │
           │  CloudLoader ◀── RecordLoader (.record, native)     │
           │      │       ◀── PcdLoader   (.pcd, fallback)       │
           │      ▼                                              │
           │  frame fusion + optional ring synthesis             │
           │      ▼                                              │
           │  LidarDetect / QRDetect / SVD solve   (UNCHANGED)   │
           │      ▼                                              │
           │  results: single_calib_result.txt (FAST-LIVO2 fmt)  │
           │           extrinsics_<cam>.yaml   (Apollo fmt)      │
           │           debug PCDs (replaces RViz publishers)     │
           └─────────────────────────────────────────────────────┘
```

Dependencies: **PCL, OpenCV, Eigen, yaml-cpp, protobuf** (protobuf only for the
record reader). Builds natively on the Orin (Ubuntu) or in a slim non-ROS
container. No Apollo runtime linkage — the record reader is self-contained.

### 2.1 Input: Apollo Cyber records, read directly in C++

- Vendor the needed protos into `proto/`:
  - `cyber/proto/record.proto` (file header / chunk / channel sections)
  - `modules/common_msgs/sensor_msgs/pointcloud.proto`
    (`apollo.drivers.PointCloud`, `PointXYZIT`: x, y, z, intensity, timestamp)
  - Copy them **from the deployed `apollo-lite` checkout** on the car
    (`/home/nvidia/01code/apollo-lite`) so field numbers match what the drivers
    actually wrote — not from upstream Apollo master.
- Implement `src/record_reader.{h,cpp}`: a minimal single-purpose reader of the
  cyber record file format (section-framed protobufs; same format the
  pure-python `cyber_record` package parses in a few hundred lines). Support
  `COMPRESS_NONE` (the default the drivers use); detect and error clearly on
  BZ2/LZ4-compressed chunks rather than silently mis-parsing.
- `RecordLoader` iterates messages on the configured channel
  (`/apollo/sensor/hesai/left/PointCloud2` etc.), **fuses up to
  `max_fusion_frames` frames** (this absorbs `record_to_pcd.py`'s job — the
  scene is static, fusion densifies azimuthally), drops NaN/zero returns, and
  fills `Common::Point{x,y,z,intensity,ring}`.
- **Ring**: `apollo.drivers.PointXYZIT` has no ring field. Config decides:
  - `beam_altitudes_deg: [14, 12, …, -16]` present → synthesize ring per point
    from elevation angle (ports `pcd_to_bag.py`'s logic; for low-line mechanical
    LiDARs like the Vanjee 16-line) → mechanical pipeline.
  - absent → `ring = 0xFFFF` → solid pipeline (AT128 / Livox, today's
    `--no-ring`).
- `PcdLoader` reads `x y z intensity [ring]` ASCII/binary PCD — keeps all
  existing captured data and the standalone-test workflow usable.
- The loader optionally writes the fused cloud to `cloud.pcd`
  (`--dump-pcd`), so `pick_roi.py`, `overlay_reproj.py`, `render_scene_qa.py`
  keep working **unmodified**.
- Livox `CustomMsg` rosbag support is dropped: on Apollo rigs the Livox driver
  already publishes an `apollo.drivers.PointCloud` channel. Delete
  `include/CustomMsg.h`, `include/CustomPoint.h`.

### 2.2 Config & CLI: yaml-cpp replaces rosparam + roslaunch

- `config/cameras/<cam>.yaml` stays the single per-camera source of truth
  (files are already valid plain YAML); parse with **yaml-cpp** into the
  existing `Params` struct. `loadParameters()` moves out of `common_lib.h` into
  `src/params.{h,cpp}` with the same defaults, no `NodeHandle`.
- New keys (all optional, defaulted): `lidar_channel`, `max_fusion_frames: 20`,
  `beam_altitudes_deg: []`, `lidar_frame`, `camera_frame`.
- Launch files are replaced by CLI (roslaunch args → flags):

  ```bash
  # was: roslaunch fast_calib calib_cam.launch cam:=cam2 scene:=scene1
  fast_calib --config config/cameras/cam2.yaml \
             --scene  calib_data/cam2/scene1 \
             --output output/cam2 [--debug-dir output/cam2/debug]

  # was: roslaunch fast_calib multi_calib_cam.launch cam:=cam2
  multi_fast_calib --config config/cameras/cam2.yaml --output output/cam2

  # was: rosrun fast_calib lidar_center_test <bag> <topic> <mode>
  lidar_center_test --config ... <record-dir-or-pcd> <channel> <solid|mech>
  ```

  `--scene` autodetects the cloud source inside the directory
  (`record/` > `cloud.pcd`); `--cloud`/`--image` override individually.
- Delete `launch/`, `package.xml`, `rviz_cfg/`.

### 2.3 Logging: tiny shim

`include/log.h` with `LOG_INFO/LOG_WARN/LOG_ERROR(fmt, ...)` printf-style
macros (reusing `color.h`), plus `LOG_*_STREAM` variants. Migration of the ~40
`ROS_*` call sites is then a mechanical rename — no message text changes, so
existing doc/tuning guidance ("a good run shows…") stays accurate.

### 2.4 Debug output: files replace RViz

- The 8 `LidarDetect` publishers, the `QRDetect` publisher, and `main.cpp`'s
  publish loop are deleted. Under `--debug-dir`, `main` instead writes each
  intermediate cloud (`filtered`, `plane`, `annulus`, `boundary`, `aligned`,
  `edge`, `centers_z0`, `centers`, `aligned_lidar_centers`, `colored_cloud`) as
  a PCD — same data, inspectable in CloudCompare/pcl_viewer, and reusing the
  debug-cloud writer pattern already in `lidar_center_test.cpp`.
- `main` now **exits after writing results** — also fixes the known cosmetic
  SIGABRT in the RViz spin loop on the car.
- Image-space QA stays with `overlay_reproj.py` / `render_scene_qa.py`
  (already ROS-free).

### 2.5 Output: add Apollo extrinsics format

Keep `single_calib_result.txt` / `multi_calib_result.txt` (FAST-LIVO2 format)
byte-compatible. Additionally write `output/<cam>/extrinsics_<cam>.yaml` in
Apollo's transform-file convention:

```yaml
header:
  frame_id: cam2            # parent: points are expressed here after transform
child_frame_id: hesai_left  # the LiDAR
transform:
  translation: {x: …, y: …, z: …}
  rotation:    {x: …, y: …, z: …, w: …}   # quaternion of R_cam_lidar
```

Semantics: `P_cam = R · P_lidar + t` (exactly today's `T_cam_lidar`), i.e. the
child-frame→frame_id transform, matching how Apollo perception consumes sensor
extrinsics files. Frame names come from the new config keys. **Verify the
direction convention against one existing extrinsics file consumed by the
apollo-lite stack on the car before finalizing** (cheap check, catastrophic if
inverted).

### 2.6 Build & packaging

- Rewrite `CMakeLists.txt`: plain CMake ≥3.16, `find_package(PCL, OpenCV,
  Eigen3, yaml-cpp, Protobuf)`, `protobuf_generate_cpp` for the vendored
  protos, three targets (`fast_calib`, `multi_fast_calib`,
  `lidar_center_test`).
- `docker/`: replace the ROS Noetic image with a slim `ubuntu:22.04` +
  `libpcl-dev libopencv-dev libyaml-cpp-dev libeigen3-dev protobuf-compiler`
  image (~⅓ the size), or document a native build on the Orin — nothing needs
  the container anymore except reproducibility.
- Scripts:
  - **delete** `scripts/pcd_to_bag.py` (raison d'être gone).
  - `capture_scene.sh`: keep RTSP grab + `cyber_recorder` steps; replace the
    convert steps with copying the record files into
    `calib_data/<cam>/<scene>/record/`. Keep `record_to_pcd.py` only as a
    standalone utility (the binary's `--dump-pcd` supersedes it in the main
    flow).
  - `multi_capture.sh`, `pick_multi_roi.sh`: swap `docker run … roslaunch …`
    invocations for direct binary calls; drop the `timeout 90`/Ctrl-C dance
    since the binary now exits.

---

## 3. Work breakdown (phased, each phase leaves a working tree)

### Phase 0 — regression baseline (before touching anything)
1. Run the current pipeline on all existing scenes with data
   (e.g. `calib_data/cam2/scene{1..3}`) in the Noetic container; archive
   `single/multi_calib_result.txt`, `circle_center_record.txt`, RMSE and
   geometry-check log lines as `test/golden/`.

### Phase 1 — de-ROS the core (input still PCD)
2. Add `include/log.h`; mechanically replace all `ROS_*` call sites.
3. Delete dead includes (`cv_bridge`, `image_geometry`, `message_filters`,
   `tf/tf.h`, `pcl_ros/*`); switch `pcl_conversions`-dependent bits to plain
   PCL.
4. Extract `Params` + yaml-cpp loader into `src/params.{h,cpp}`; add CLI
   parsing (getopt or a ~50-line hand-rolled parser) to all three mains.
5. Replace `DataPreprocess` internals with `PcdLoader` (+ ring synthesis);
   remove publishers from `QRDetect`/`LidarDetect` constructors (drop the
   `NodeHandle` params); rewrite `main.cpp` tail: debug-PCD dump + clean exit.
   Same surgery on `lidar_center_test.cpp` and `multi_scene.cpp` (the latter
   only needs params/logging — its solver is already ROS-free).
6. New plain `CMakeLists.txt`; delete `package.xml`, `launch/`, `rviz_cfg/`,
   `include/CustomMsg.h`, `include/CustomPoint.h`.
7. **Gate:** build natively (macOS/Ubuntu, no ROS anywhere); run on golden
   scenes' `cloud.pcd`; extrinsics match Phase-0 baseline (rotation < 0.05°,
   translation < 1 mm — PCL RANSAC is randomized, so compare within tolerance,
   or seed `pcl::SampleConsensusModel` for exactness).

### Phase 2 — native Apollo record input
8. Vendor protos from the car's apollo-lite; implement `record_reader` +
   `RecordLoader` with fusion, NaN/zero filtering, `--dump-pcd`.
9. **Gate:** for one captured scene, loader parity vs the python path — same
   channel, same `max_fusion_frames`: point count and fused cloud match
   `record_to_pcd.py` output; then full calibration on records end-to-end.
10. Update `capture_scene.sh` / `multi_capture.sh` / `pick_multi_roi.sh`.

### Phase 3 — Apollo output + packaging
11. Apollo extrinsics YAML writer (+ convention check against a real consumed
    file on the car); frame-name config keys.
12. Slim Docker image or documented native Orin build; delete the Noetic
    docker assets.
13. Update `README.md`, `docs/lidar2camera_calibration_guide*.md`,
    `scripts/README_lidar2cam_capture.md` (sections 1.2, 3.x, 4.2–4.4 change
    substantially — no more container/roslaunch/bag).

### Phase 4 — field validation
14. On car-ningde-orin: fresh capture → calibrate one camera end-to-end from
    records; compare against the accepted `cam2` multi-scene result (joint
    RMSE ≈ 9.6 mm) and QA overlays.

---

## 4. Risks & open questions

| Risk | Mitigation |
|---|---|
| Cyber record chunk compression (BZ2/LZ4) or format drift in apollo-lite | Vendor protos from the deployed checkout; explicit error on compressed chunks; `record_to_pcd.py` + `PcdLoader` remains a fully supported fallback path, so Phase 2 can never block calibration |
| Apollo extrinsics direction convention inverted | Verify against a file the on-car stack already consumes; assert `P_cam = T · P_lidar` in the file header comment |
| PCL RANSAC nondeterminism breaks exact regression | Compare within physical tolerance, or fix the RNG seed in test builds |
| Existing YAMLs relied on rosparam quirks (`subst_value`, flat namespace) | They're plain scalars at root — yaml-cpp reads them as a map directly; add a startup dump of all resolved params for eyeballing |
| Intensity scale differences (Apollo uint intensity vs float) | Loader casts to float unchanged — same values `record_to_pcd.py` produced, thresholds (Otsu/percentile) are adaptive anyway |

Out of scope (unchanged): detection algorithms and their tuning constants,
board geometry, camera intrinsics workflow, python QA scripts' internals.
