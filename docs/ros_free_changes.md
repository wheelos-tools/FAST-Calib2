# FAST-Calib2 is now ROS-free and Apollo-native — what changed and how to use it

*Branch `feat/ros-free-apollo` · 2026-08-18/19 · github.com/wheelos-tools/FAST-Calib2 · [中文版本在下方](#fast-calib2-已-ros-free-并原生对接-apollo--改了什么怎么用)*

LiDAR↔camera extrinsic calibration now reads **Apollo Cyber records natively**
and builds **as a standard Apollo module with bazel** — no roslaunch, no
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
- **Builds as a standard Apollo module.** Copy the repo to
  `<apollo>/modules/calibration/fast_calib` and run
  `bash apollo.sh build_opt calibration/fast_calib` in the dev container —
  scoped to this module, using the workspace's own bazel third-party modules
  (`@local_config_pcl` 1.15, `@opencv` 4.13, `@eigen`). Binaries are used
  straight from `bazel-bin`. `scripts/apollo_build.sh` automates
  copy/build/collect; a plain `CMakeLists.txt` remains as a fallback for
  machines without an Apollo checkout.
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
# put the code inside the Apollo workspace and build (inside the dev container)
cp -r FAST-Calib2 $APOLLO/modules/calibration/fast_calib
bash apollo.sh build_opt calibration/fast_calib

# run straight from bazel-bin (inside the container, from the module dir)
cd /apollo/modules/calibration/fast_calib
FC=/apollo/bazel-bin/modules/calibration/fast_calib

# single scene — reads calib_data/<cam>/<scene>/{image.png, record/ | cloud.pcd}
$FC/fast_calib --config config/cameras/cam2.yaml --scene calib_data/cam2/scene1 --output output/cam2

# joint fit after ≥3 scenes
$FC/multi_fast_calib --config config/cameras/cam2.yaml --output output/cam2
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
variables are gone. Full walkthroughs: `docs/simple_guide.md` and
`docs/lidar2camera_calibration_guide.md` (中文: `_zh.md`).

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
| Fresh-clone guide walkthrough (2026-08-19) | clone → apollo.sh build → calibrate reproduced the same joint T; every documented command validated verbatim |

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
4. **Copy data into the Apollo module dir, don't symlink** — execution now
   happens inside the dev container, and symlinks to host-only paths dangle
   there.

### Still open

- Field re-validation on car-ningde-orin with a fresh record-native capture.
- Compressed (BZ2/LZ4) records are rejected by design — cyber_recorder's
  default is uncompressed.

Full plan-vs-implementation record: `docs/refactor_plan_ros_free_apollo.md`.

---

# FAST-Calib2 已 ROS-free 并原生对接 Apollo —— 改了什么、怎么用

*分支 `feat/ros-free-apollo` · 2026-08-18/19 · github.com/wheelos-tools/FAST-Calib2*

激光雷达↔相机外参标定现在**直接读取 Apollo Cyber record**，并**作为标准 Apollo
模块用 bazel 编译** —— 没有 roslaunch、没有 bag、没有 catkin、没有 ROS 容器。
检测与求解算法完全未动，重构后**逐位复现**了 7 月的黄金标定结果。

## 为什么做这件事

我们的数据一直产自 Apollo，但旧程序只认 ROS bag —— 每次采集都要经过两个转换
脚本，其中一个还必须跑在一个只为此存在的 ROS Noetic 容器里。而 C++ 里的 ROS
纯属管道：rosparam 读配置、`ROS_INFO` 打日志、RViz publisher 做调试，外加一个
每次成功后都会 SIGABRT 崩溃的无限循环。

```
之前: cyber_recorder .record → record_to_pcd.py → pcd_to_bag.py (ROS!) → cloud.bag → roslaunch fast_calib
之后: cyber_recorder .record → fast_calib → T_cam_lidar + Apollo 外参 YAML
```

## 改了什么

- **原生读取 record。** `src/cyber_record_reader.hpp` 用约 370 行的 protobuf
  *wire-format* 解码器解析 cyber record 中的 `apollo.drivers.PointCloud` ——
  不依赖 libprotobuf，也不链接 Apollo 运行时；`proto/*.proto` 仅作字段号参考。
  多帧融合（原 `record_to_pcd.py`）与扫描线 ring 合成（原 `pcd_to_bag.py`）
  移入加载器；PCD 输入保留作后备并兼容旧数据。
- **作为标准 Apollo 模块编译。** 把代码复制到
  `<apollo>/modules/calibration/fast_calib`，在 dev 容器内执行
  `bash apollo.sh build_opt calibration/fast_calib` —— 仅编译本模块，使用
  工作空间自带的三方模块（`@local_config_pcl` 1.15、`@opencv` 4.13、
  `@eigen`），可执行文件直接从 `bazel-bin` 使用。`scripts/apollo_build.sh`
  可自动化拷贝/编译/收集；无 Apollo 的机器可用 `CMakeLists.txt` 后备编译。
- **配置与命令行。** 同样的扁平相机 YAML 由内置小解析器读取（无 yaml-cpp）；
  launch 文件变成参数：`--config`、`--scene`、`--output`。场景目录里的
  `cloud_roi.txt` 会自动应用，逐场景手动 ROI 不再需要改公共配置。
- **调参移出源码。** 过去要改 `lidar_detect.hpp` 再重编译的平面阈值与聚类
  常数，现在是配置键（`annulus_cluster_tolerance`、
  `board_plane_inlier_threshold` 等），默认值即原常数。每台设备的调参写在
  该设备的相机 YAML 里。
- **调试。** RViz 话题移除；`--debug-dir` 把每个中间点云写成 PCD（可用
  CloudCompare 打开）。程序运行结束即退出，退出码有意义（0 成功、1 加载
  失败、2 检测失败）—— 顺带修掉了旧的收尾 SIGABRT，编排脚本也不再需要
  timeout/kill。
- **输出。** 在不变的 FAST-LIVO2 txt 之外，新增 Apollo 格式外参
  （`single/multi_calib_extrinsics.yaml`），方向遵循 Apollo 自己的相机外参
  惯例（对照 apollo-base 的 `front_6mm_extrinsics.yaml` 等文件核实）：
  `frame_id` = **雷达**（父），`child_frame_id` = **相机**，transform =
  相机在雷达坐标系中的位姿（`P_lidar = R·P_cam + t`，即标定出的
  `T_cam_lidar` 的逆），四元数按 `x, y, z, w`。可直接放入 perception 参数目录。
- **新版 OpenCV 的 ArUco。** Apollo 的 bazel OpenCV 是 4.13、无 contrib，
  ArUco 在 `objdetect` 里且 API 变了。`qr_detect.hpp` 现在两者都支持：
  OpenCV ≥ 4.7 走新 API（`ArucoDetector`、逐标记 `solvePnP` 替代
  `estimatePoseSingleMarkers`、`matchImagePoints`+`solvePnP` 替代
  `estimatePoseBoard`、`drawFrameAxes`），旧版本仍走 contrib 路径。

**刻意保持不变：** 环提取、同心圆拟合、ArUco 板几何、SVD 求解、所有调参
默认值、两种结果 txt 格式、以及 python QA 脚本（`pick_roi.py`、
`overlay_reproj.py`、`render_scene_qa.py`）。

## 现在怎么用

```bash
# 把代码放入 Apollo 工作空间并编译（在 dev 容器内）
cp -r FAST-Calib2 $APOLLO/modules/calibration/fast_calib
bash apollo.sh build_opt calibration/fast_calib

# 直接从 bazel-bin 运行（容器内、模块目录下）
cd /apollo/modules/calibration/fast_calib
FC=/apollo/bazel-bin/modules/calibration/fast_calib

# 单场景 —— 读取 calib_data/<cam>/<scene>/{image.png, record/ | cloud.pcd}
$FC/fast_calib --config config/cameras/cam2.yaml --scene calib_data/cam2/scene1 --output output/cam2

# ≥3 个场景后做联合求解
$FC/multi_fast_calib --config config/cameras/cam2.yaml --output output/cam2
```

配置里指向你的 Apollo 通道；仅低线束机械雷达需要波束表：

```yaml
lidar_channel: "/apollo/sensor/hesai/main_front/PointCloud2"
lidar_frame: "hesai_main_front"     # 写入 Apollo 外参 YAML 的坐标系名
camera_frame: "cam2"
max_fusion_frames: 0                # 0 = 融合整个 record（场景静止）
beam_altitudes_deg: [14, 12, ...]   # AT128 / Livox 省略（固态流程）
```

`capture_scene.sh` 现在把原始 record 存进场景目录（另存一份 `cloud.pcd`
供选 ROI）；`multi_capture.sh` 与 `pick_multi_roi.sh` 直接驱动可执行文件。
`TOPIC` / `FRAME` / `RING_FLAG` 环境变量已移除。完整流程见
`docs/simple_guide.md` 与 `docs/lidar2camera_calibration_guide_zh.md`。

## 证明它仍然标得准

与 7 月黄金结果的回归对比（wheel101 · AT128 + 海康相机 · 3 场景，该机的
调参通过新配置键提供）：

| 检查项 | 结果 |
|---|---|
| record 解码器 vs python `cyber_record` | 1,229,230 点 / 47 帧 —— 完全一致 |
| 逐场景 RMSE（PCD 输入） | 4.2 / 3.5 / 7.3 mm —— 与黄金值完全相同 |
| 全部 24 个环心 | 与黄金 `circle_center_record.txt` 完全一致 |
| 多场景联合 `T_cam_lidar` | 与黄金 `multi_calib_result.txt` **逐位一致** |
| 原始 record 路径（425 MB `rec.00000`，无 PCD/bag） | 融合 48 帧 · scene1 RMSE 2.8 mm |
| Apollo 编译的可执行文件（bazel，OpenCV 4.13 / PCL 1.15） | 逐场景 RMSE 4.3 / 3.2 / 7.4 mm；联合 T 与黄金差 ~0.05° / 0.13 mm（新 ArUco API 带来的相机侧微小差异） |
| 全新克隆按指南走通（2026-08-19） | clone → apollo.sh 编译 → 标定，复现同一联合 T；文档中每条命令逐字验证 |

record 路径的结果甚至比旧流程*更好*（scene1 2.8 mm vs 4.2 mm），因为它融合
了全部 48 帧，而旧的 PCD 转换只保留 20 帧。

## 已有设备或在途数据怎么办

1. **旧采集场景仍可用。** 已有的 `cloud.pcd` 是一等输入；只有 `cloud.bag`
   彻底作废。
2. **每台设备的调参：** 若你的机器曾在 `lidar_detect.hpp` 上打过本地补丁，
   把那些数字搬进相机 YAML（见新的 `*_threshold` / `*_cluster_*` 键）。
   仓库默认值对应 car-ningde 的 AT128；wheel101 的数值在其未跟踪的配置里。
3. **外参方向 —— 已核实并修正。** 第一版把相机当父坐标系；对照 apollo-base
   自带的相机↔雷达外参文件后确认 Apollo 存的是*相机在雷达坐标系中的位姿*，
   现已按该惯例输出 —— 但 `d03e26f` 之前生成的文件方向相反：请重新生成。
4. **数据用复制放进 Apollo 模块目录，不要用软链接** —— 现在运行发生在 dev
   容器内，指向主机路径的软链接在容器里会失效。

### 仍待办

- 在 car-ningde-orin 上用全新的 record 原生采集做实地复验。
- 压缩（BZ2/LZ4）record 按设计拒绝 —— cyber_recorder 默认不压缩。

完整的计划-实现对照记录见 `docs/refactor_plan_ros_free_apollo.md`。
