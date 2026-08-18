# ROS-free FAST-Calib2 on Apollo Data — Plan & Implementation Record

Branch: `feat/ros-free-apollo`. Goal: remove every ROS dependency (roscpp,
catkin, rosbag, rosparam, RViz, the ROS side of the calibration container) and
make **Apollo Cyber RT data the native input**: point clouds straight from
`cyber_recorder` `.record` files (`apollo.drivers.PointCloud`), config from
plain YAML, results additionally in Apollo extrinsics YAML format. The
detection/solve algorithms are untouched.

**Status: implemented and validated (2026-08-18).** This document was written
as the plan and then updated to record what was actually built, where the
implementation deviated from the plan and why, and what remains open.

---

## 1. Why (unchanged from the plan)

The calibration math (annulus extraction, concentric fitting, ArUco board
pose, SVD solve) was always pure **PCL + OpenCV + Eigen**; ROS was only
plumbing — rosbag as the input container, rosparam/roslaunch for config,
`ROS_*` logging, RViz debug publishers plus an infinite spin loop (source of
the known post-result SIGABRT), and a catkin build that forced every run
through the `fast-calib2:noetic` container.

Worse, on the actual rigs the data already originated in Apollo, so the old
chain did a pointless round-trip — the ROS bag existed only because the binary
demanded one:

```
cyber_recorder .record ─record_to_pcd.py─▶ cloud.pcd ─pcd_to_bag.py (ROS!)─▶ cloud.bag ─▶ C++
```

## 2. What was built

```
calib_data/<cam>/<scene>/
    image.png            (ffmpeg RTSP grab, unchanged)
    record/rec.00000     (raw cyber_recorder output, copied as-is)
    cloud.pcd            (viewing/ROI-picking copy; also a valid input)
    cloud_roi.txt        (optional; pick_roi.py output, auto-applied)

           ┌─────────────────────────────────────────────────────┐
           │ fast_calib (plain C++17 binary, no ROS)             │
           │  DataPreprocess ◀── cyber record file(s)  (native)  │
           │                 ◀── cloud.pcd             (fallback)│
           │      ▼  frame fusion + optional ring synthesis      │
           │  LidarDetect / QRDetect / SVD solve   (UNCHANGED)   │
           │      ▼                                              │
           │  single/multi_calib_result.txt   (FAST-LIVO2 fmt)   │
           │  single/multi_calib_extrinsics.yaml (Apollo fmt)    │
           │  --debug-dir PCD dumps (replaces RViz topics)       │
           └─────────────────────────────────────────────────────┘
```

### 2.1 Input: Apollo Cyber records, read directly in C++

- `src/cyber_record_reader.hpp` walks the record container (16-byte section
  headers; header section padded to 2048 bytes; Channel / ChunkHeader /
  ChunkBody sections) and decodes `SingleMessage` → `apollo.drivers.PointCloud`
  → `PointXYZIT`. NaN and zero returns are dropped (same policy as the old
  python extraction). Only `COMPRESS_NONE` records are accepted; compressed
  chunks fail loudly.
- **Deviation from plan:** the plan called for vendored protos compiled with
  libprotobuf. Instead the ~370-line reader decodes the protobuf **wire format
  directly**, so no protobuf toolchain (or Apollo runtime) is needed in any
  build environment. The vendored `proto/record.proto` and
  `proto/pointcloud.proto` (copied from the deployed apollo-lite checkout) are
  kept as the authoritative field-number reference.
- Frame fusion moved into the loader (absorbing `record_to_pcd.py`'s job).
  `max_fusion_frames` defaults to **0 = fuse every frame in the record**
  (deviation: the plan said 20; real captures are short static recordings and
  were in practice fused wholesale, and more frames measurably helped).
- Ring synthesis from a `beam_altitudes_deg` table in the config (absorbing
  `pcd_to_bag.py`'s job): table present → ring per point by elevation →
  mechanical pipeline; absent → solid pipeline. Same rule the bag path
  applied, minus the bag.
- PCD input remains fully supported (a `ring` column is honored if present);
  `--dump-pcd` writes the fused input cloud so `pick_roi.py` /
  `overlay_reproj.py` / `render_scene_qa.py` keep working unmodified.
- Livox `CustomMsg` rosbag support dropped (`include/CustomMsg.h`,
  `include/CustomPoint.h` deleted) — Livox units publish an Apollo
  `PointCloud2` channel on our rigs.

### 2.2 Config & CLI

- `include/params.h`: the `Params` struct plus a loader for the flat
  "key: value" per-camera YAML. **Deviation:** a ~100-line self-contained
  parser instead of yaml-cpp — the configs are flat scalar maps, and this
  keeps the dependency list at exactly PCL/OpenCV/Eigen. roslaunch-style
  `$(find fast_calib)` in path values is still substituted (repo root inferred
  from the config location); the legacy `bag_path` key is accepted as
  `cloud_path`.
- New keys: `cloud_path`, `lidar_channel` (falls back to `lidar_topic`),
  `max_fusion_frames`, `beam_altitudes_deg`, `lidar_frame`, `camera_frame`.
- **Addition beyond the plan:** the LiDAR-detector tuning constants that the
  guide told users to edit in `lidar_detect.hpp` and rebuild are now config
  keys with the previous values as defaults: `board_plane_inlier_threshold`
  (0.07), `annulus_plane_inlier_threshold` (0.07),
  `boundary_plane_inlier_threshold` (0.03), `annulus_cluster_tolerance` (0.10)
  / `annulus_cluster_min_size` (30), `mech_cluster_tolerance` (0.09) /
  `mech_cluster_min_size` (80), `auto_roi_cluster_min_size` (200). Motivation:
  the wheel101 golden results turned out to have been produced with
  *uncommitted source tweaks* to exactly these numbers — per-rig tuning now
  lives in the per-camera YAML instead of source patches.
- CLI replaces the launch files (`launch/` deleted):

  ```bash
  fast_calib --config config/cameras/<cam>.yaml \
             --scene  calib_data/<cam>/<scene> \
             [--output output/<cam>] [--debug-dir <dir>] [--dump-pcd <path>]
             [--cloud <path>] [--image <path>] [--no-roi-file]
  multi_fast_calib --config config/cameras/<cam>.yaml [--output <dir>]
  lidar_center_test [--config <yaml>] <pcd|record file|record dir> <channel> [auto|solid|mech]
  ```

  `--scene` resolves `image.png` plus the cloud source (`record/` preferred,
  else `cloud.pcd`) and **auto-applies `cloud_roi.txt`** when present (forces
  manual-ROI mode) — this reproduces per-scene manual ROIs without mutating
  the shared config between scenes, which is what the old flow did.

### 2.3 Logging, debug output, process behavior

- `include/log.h`: `LOG_INFO/WARN/ERROR(_STREAM)` printf/stream macros
  (mechanical rename of ~85 `ROS_*` call sites; message texts unchanged, so
  existing troubleshooting docs and the orchestrator scripts' `grep`s still
  match).
- The 8 RViz publishers in `LidarDetect`, the one in `QRDetect`, and
  `main.cpp`'s publish loop are gone. `--debug-dir` writes every intermediate
  cloud (`filtered`, `plane`, `annulus`, `boundary`, `aligned`, `edge`,
  `center_z0`, `qr_centers`, `lidar_centers`, `aligned_lidar_centers`,
  `colored_cloud`) as a PCD.
- Binaries **exit** after writing results (fixes the RViz-loop SIGABRT) with
  meaningful codes — 0 success, 1 load/config failure, 2 detection failure
  (fewer than 4+4 centers). The orchestrator scripts dropped their
  poll/`docker kill`/timeout dances accordingly.

### 2.4 Output: Apollo extrinsics format

`src/apollo_extrinsics.hpp` writes, next to the FAST-LIVO2 txt files
(byte-compatible, unchanged):

- `single_calib_extrinsics.yaml` (from `fast_calib`)
- `multi_calib_extrinsics.yaml` (from `multi_fast_calib`)

(Deviation: the plan said `extrinsics_<cam>.yaml`; the binaries don't know the
camera label, so the names parallel the txt outputs instead.) Format and
direction follow Apollo's own camera↔LiDAR extrinsics files (verified against
`front_6mm_extrinsics.yaml` and the velodyne↔novatel examples in apollo-base):
`header.frame_id` = **LiDAR** (parent), `child_frame_id` = **camera**, and the
transform is the camera's pose in the LiDAR frame — the inverse of the
calibrated `T_cam_lidar` — as translation + scalar-last quaternion (`x,y,z,w`).
Frame names come from the new config keys.

### 2.5 Build & packaging

- Plain CMake (≥3.10, C++17): PCL + OpenCV + Eigen3, three targets.
  `package.xml`, catkin, and `rviz_cfg/` are gone.
- **Gotcha discovered at link time (now handled):** `Common::Point` is a
  custom point type, so `PCL_NO_PRECOMPILE` must be in effect before *any* PCL
  header in every TU — the old code only worked by include-order luck.
  CMake now defines it globally.
- The `fast-calib2:noetic` image is still used **as a convenient prebuilt
  PCL/OpenCV environment** (`docker/build.sh` now runs cmake instead of
  catkin_make; `docker/run.sh` / `run_multi.sh` invoke the binaries directly).
  The planned slim non-ROS image was **not** built — nothing needs ROS at
  runtime anymore, so replacing the base image is a size/cosmetic cleanup,
  left as an open item. Any Ubuntu box with `libpcl-dev libopencv-dev
  libeigen3-dev` builds natively.
- Scripts: `pcd_to_bag.py` deleted. `capture_scene.sh` now keeps the raw
  record in `calib_data/<cam>/<scene>/record/` and still produces `cloud.pcd`
  for ROI picking/QA (`record_to_pcd.py` retained for that and as a
  cross-check). `multi_capture.sh` / `pick_multi_roi.sh` call the binaries
  directly; the `TOPIC`/`FRAME`/`RING_FLAG` env vars are gone (ring handling
  moved to the config).

## 3. Validation (wheel101, 2026-08-18)

Golden baseline: the July 2026 `cam157_at128_20260703` calibration (Hesai
AT128 + Hikvision, 3 scenes, manual per-scene ROI) — archived results in
`output/cam157_at128_20260703/` on nvidia@10.0.39.101, raw records preserved
under `/home/nvidia/01projects/apollo/data/calib_capture/`. The regression ran
with that machine's tuning values (plane gates 0.08, annulus cluster 0.06/30,
mech 0.13/20) supplied via the new config keys.

| Check | Result |
|---|---|
| Record-reader parity vs python `cyber_record` (Vanjee capture) | identical fused point count: 1,229,230 pts / 47 frames |
| Per-scene single-scene RMSE (PCD input, 3 scenes) | 4.2 / 3.5 / 7.3 mm — exactly the golden report values |
| All 24 extracted centers vs golden `circle_center_record.txt` | identical |
| Joint multi-scene `T_cam_lidar` vs golden `multi_calib_result.txt` | **bit-identical** (all printed decimals) |
| Native record input (raw 425 MB `rec.00000`, no PCD/bag) | end-to-end pass; 48 frames fused (3.5M pts), scene1 RMSE 2.8 mm (tighter than the 20-frame PCD's 4.2 mm) |

The plan budgeted a tolerance (rotation < 0.05°, translation < 1 mm) for PCL
RANSAC nondeterminism; in practice the pipeline is deterministic on identical
input and the match was exact, so no seeding work was needed.

Planned-but-changed in testing: Phase 0 ("re-run the old pipeline to produce a
baseline") was unnecessary — the archived July outputs *are* the baseline. The
macOS native-build gate was skipped (no PCL installed locally); instead the
ROS-free-only headers (record reader, params) were compile-tested and
data-validated on macOS, and the full build ran in the container.

## 4. Open items

1. ~~Verify the Apollo extrinsics direction convention~~ **Done (2026-08-18):**
   checked against `front_6mm_extrinsics.yaml`, the velodyne↔novatel examples,
   and the wheelflow camera↔lidar file in
   `/home/nvidia/01projects/apollo-base`. The first version of the writer had
   the parent/child direction inverted relative to Apollo's camera-extrinsics
   convention; the writer now emits LiDAR-as-parent / camera-as-child with the
   inverted transform, matching those files structurally and semantically.
2. **Docs debt:** `docs/lidar2camera_calibration_guide.md` (+ zh),
   `scripts/README_lidar2cam_capture.md`, `SETUP_5CAM_VANJEE.md`, and
   `docker/README.md` still describe the ROS-bag/roslaunch flow. README.md is
   updated; the guides need a pass.
3. **Slim container image** (or documented native Jetson build) to replace
   `fast-calib2:noetic` as the build environment.
4. **Field validation on car-ningde-orin** (Phase 4): fresh capture →
   record-native calibration, compared against the accepted cam2 joint result.
   Note the committed tuning defaults are the car values; wheel101 overrides
   them in its (untracked) per-camera configs — e.g. `cam157_regress.yaml`.
5. BZ2/LZ4-compressed records are rejected, not supported (cyber_recorder's
   default is uncompressed; revisit only if a rig turns on compression).
