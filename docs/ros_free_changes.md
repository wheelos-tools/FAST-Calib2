# FAST-Calib2 is now ROS-free and Apollo-native — what changed and how to use it

*Branch `feat/ros-free-apollo` · 2026-08-18 · github.com/wheelos-tools/FAST-Calib2*

LiDAR↔camera extrinsic calibration now reads **Apollo Cyber records natively**
and builds **inside the Apollo dev environment with bazel** — no roslaunch, no
bags, no catkin, no ROS container. The detection and solve algorithms are
untouched, and the refactor reproduces our July golden calibration
**bit-for-bit**.

---

## Why we did this

Our data has always originated in Apollo, but the binary demanded a ROS bag —
so every capture took a round-trip through two conversion scripts, one of which
had to run inside a ROS Noetic container that existed for no other reason.
Meanwhile ROS inside the C++ was pure plumbing: rosparam for config,
`ROS_INFO` for logging, RViz publishers for debugging, and an infinite spin
loop that crashed with SIGABRT after every successful run.

```
Before: cyber_recorder .record → record_to_pcd.py → pcd_to_bag.py (ROS!) → cloud.bag → roslaunch fast_calib
After:  cyber_recorder .record → fast_calib → T_cam_lidar + Apollo extrinsics YAML
```

## What changed

- **Native record input.** `src/cyber_record_reader.hpp` parses cyber record
  files and decodes `apollo.drivers.PointCloud` with a ~370-line protobuf
  *wire-format* decoder — no libprotobuf, no Apollo runtime linkage. The
  vendored `proto/*.proto` files are the field-number reference. Frame fusion
  (was `record_to_pcd.py`) and scan-ring synthesis (was `pcd_to_bag.py`) moved
  into the loader; PCD input remains as a fallback and for old data.
- **Builds under Apollo.** `scripts/apollo_build.sh` syncs the sources to
  `<apollo>/modules/calibration/fast_calib` and builds with the workspace's
  own bazel third-party modules (`@local_config_pcl` 1.15, `@opencv` 4.13,
  `@eigen`) inside the running `apollo_dev` container. A plain `CMakeLists.txt`
  (PCL/OpenCV/Eigen) remains as a fallback for machines without an Apollo
  checkout.
- **Config & CLI.** The same flat per-camera YAML is read by a small built-in
  parser (no yaml-cpp); launch files became flags: `--config`, `--scene`,
  `--output`. A `cloud_roi.txt` in the scene folder is applied automatically,
  so per-scene manual ROIs no longer mutate the shared config.
- **Tuning moved out of the source.** The plane-gate and cluster constants we
  used to patch in `lidar_detect.hpp` and rebuild are now config keys
  (`annulus_cluster_tolerance`, `board_plane_inlier_threshold`, …) with the
  old values as defaults. Per-rig tuning belongs in that rig's camera YAML.
- **Debugging.** RViz topics are gone; `--debug-dir` writes every intermediate
  cloud as a PCD you can open in CloudCompare. Binaries exit when done — with
  real exit codes (0 ok, 1 load error, 2 detection failed) — which also kills
  the old post-result SIGABRT and let the orchestrator scripts drop their
  timeout/kill dances.
- **Output.** Alongside the unchanged FAST-LIVO2 txt files, both solvers write
  Apollo-format extrinsics (`single_calib_extrinsics.yaml` /
  `multi_calib_extrinsics.yaml`) in Apollo's own camera-extrinsics convention —
  verified against `front_6mm_extrinsics.yaml` and the velodyne↔novatel
  examples in apollo-base: `frame_id` = **LiDAR** (parent), `child_frame_id` =
  **camera**, transform = camera pose in the LiDAR frame
  (`P_lidar = R·P_cam + t`, the inverse of the calibrated `T_cam_lidar`),
  quaternion keyed `x, y, z, w`. The file drops into perception params as-is.
- **ArUco on modern OpenCV.** Apollo's bazel OpenCV is 4.13 without contrib,
  where ArUco lives in `objdetect` with a changed API. `qr_detect.hpp` now
  supports both: the new value-based API (`ArucoDetector`, per-marker
  `solvePnP` replacing `estimatePoseSingleMarkers`,
  `matchImagePoints`+`solvePnP` replacing `estimatePoseBoard`,
  `drawFrameAxes`) on OpenCV ≥ 4.7, the legacy contrib path on older builds.

**Unchanged on purpose:** annulus extraction, concentric fitting, ArUco board
geometry, SVD solve, all tuning defaults, both result txt formats, and the
python QA scripts (`pick_roi.py`, `overlay_reproj.py`, `render_scene_qa.py`).

## How you run it now

```bash
# build inside the Apollo dev environment (bazel; APOLLO_C defaults to the
# first running apollo_dev container)
APOLLO_HOST=/path/to/apollo scripts/apollo_build.sh

# single scene — reads calib_data/<cam>/<scene>/{image.png, record/ | cloud.pcd}
./build/fast_calib --config config/cameras/cam2.yaml \
                   --scene calib_data/cam2/scene1 --output output/cam2

# joint fit after ≥3 scenes
./build/multi_fast_calib --config config/cameras/cam2.yaml --output output/cam2
```

Point the config at your Apollo channel and, only for low-line mechanical
LiDARs, give it the beam table:

```yaml
lidar_channel: "/apollo/sensor/hesai/main_front/PointCloud2"
lidar_frame: "hesai_main_front"     # names used in the Apollo extrinsics YAML
camera_frame: "cam2"
max_fusion_frames: 0                # 0 = fuse the whole record (scene is static)
beam_altitudes_deg: [14, 12, ...]   # omit for AT128 / Livox (solid pipeline)
```

`capture_scene.sh` now stores the raw record in the scene folder (plus a
`cloud.pcd` for ROI picking); `multi_capture.sh` and `pick_multi_roi.sh` drive
the binaries directly. The `TOPIC` / `FRAME` / `RING_FLAG` environment
variables are gone.

## Proof it still calibrates

Regression vs the July golden run (wheel101 · AT128 + Hikvision · 3 scenes,
that machine's tuning via the new config keys):

| Check | Result |
|---|---|
| Record reader vs python `cyber_record` | 1,229,230 pts / 47 frames — identical |
| Per-scene RMSE (PCD input) | 4.2 / 3.5 / 7.3 mm — exactly the golden values |
| All 24 extracted ring centers | identical to golden `circle_center_record.txt` |
| Joint multi-scene `T_cam_lidar` | **bit-identical** to golden `multi_calib_result.txt` |
| Raw-record path (425 MB `rec.00000`, no PCD/bag) | 48 frames fused · scene1 RMSE 2.8 mm |
| Apollo-built binaries (bazel, OpenCV 4.13 / PCL 1.15) | per-scene RMSE 4.3 / 3.2 / 7.4 mm; joint T within ~0.05° / 0.13 mm of golden (tiny camera-side deltas from the new ArUco API) |

The record path actually came out *tighter* than the old flow (2.8 mm vs
4.2 mm on scene1) because it fuses all 48 frames instead of the 20 the old PCD
conversion kept.

## If you have a rig or data in flight

1. **Old captured scenes keep working.** Existing `cloud.pcd` files are a
   first-class input; only `cloud.bag` files are now dead weight.
2. **Per-rig tuning:** if your machine carried local edits to
   `lidar_detect.hpp`, move those numbers into your camera YAML (see the new
   `*_threshold` / `*_cluster_*` keys). Committed defaults match the
   car-ningde AT128 values; wheel101's values live in its untracked configs.
3. **Extrinsics direction — verified and fixed.** The first cut of the writer
   had camera as the parent frame; checking against apollo-base's own
   camera↔LiDAR files showed Apollo stores the *camera pose in the LiDAR
   frame*. The writer now emits that convention — but files generated before
   commit `d03e26f` carry the inverse transform: regenerate them.

### Still open

- The long-form calibration guides (`docs/lidar2camera_calibration_guide*.md`,
  capture-script README) still describe the ROS flow; README.md is updated.
- Field re-validation on car-ningde-orin with a fresh record-native capture.
- Compressed (BZ2/LZ4) records are rejected by design — cyber_recorder's
  default is uncompressed.

Full plan-vs-implementation record: `docs/refactor_plan_ros_free_apollo.md`.
