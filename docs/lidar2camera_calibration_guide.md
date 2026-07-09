# LiDAR ↔ Camera Extrinsic Calibration — Detailed Guide (FAST-Calib2)

A complete, reproducible procedure for calibrating the rigid transform between a **camera** and a
**LiDAR** using FAST-Calib2's reflective-annular-target pipeline. Covers: (1) environment setup,
(2) data capture, (3) formatting the data, (4) calibration, (5) evaluation.

Applies to any camera (RTSP / USB / recording) and any LiDAR (mechanical multi-line, solid-state,
or Livox), whether point clouds come from a native ROS driver or **Apollo Cyber RT**.

**Result:** `T_camera←lidar` = `(R, t)` such that `P_cam = R · P_lidar + t`.

---

## 0. How FAST-Calib2 works (mental model)

- It runs **offline** on one `(image.png, cloud.bag)` pair per *scene* (one board placement).
- The **camera** side detects 4 ArUco markers → board pose → the 4 ring centers (3D).
- The **LiDAR** side extracts the 4 reflective-annulus centers (3D) directly from the cloud.
- It solves the rigid transform that best aligns the two sets of 4 centers (SVD).
- **Single scene** = one placement (sensitive). **Multi-scene** = ≥3 placements combined (robust).

Placeholders used below: `<REPO>` = FAST-Calib2 root, `<CAM>` = a camera label, `<SCENE>` = a
board placement label, `<IMG>` = the ROS container image `fast-calib2:noetic`.

---

# 1. Install & set up the environment

### 1.1 Prerequisites
- A machine that can reach the camera and LiDAR (the "host").
- **Docker** (for the ROS Noetic build container — no ROS needed on the host).
- **ffmpeg** (RTSP frame grab), **git**, **python3** with `pip`.
- A **reflective annular calibration target** (FAST-Calib2 board: 4 ArUco markers `DICT_6X6_250`
  + 4 retro-reflective annuli). Measure and record its real dimensions.

### 1.2 Get FAST-Calib2 and build the container
```bash
git clone https://github.com/wheelos-tools/FAST-Calib2.git <REPO>
cd <REPO>
docker/build.sh        # builds fast-calib2:noetic and compiles the workspace into docker/.ws_devel
```
The container carries PCL + OpenCV + ROS Noetic; compiled binaries persist in `docker/.ws_devel/`
across `docker run --rm`. Re-run `docker/build.sh` (or just `catkin_make` in the container) after
any C++ change.

### 1.3 Host Python dependencies (for the helper scripts)
```bash
# reading Apollo Cyber records (Apollo/Cyber LiDAR source only):
python3 -m pip install --user cyber_record "protobuf==3.19.4"
# ROI picker + QA rendering:
python3 -m pip install --user open3d opencv-python numpy
```
> If your shell has **conda** active, its `python3` may lack these. Point the scripts at the
> system interpreter with `PYTHON=/usr/bin/python3` (they honor that env var).

### 1.4 Helper scripts (`<REPO>/scripts/`)
| Script | Role |
|---|---|
| `capture_scene.sh` | grab a camera frame + record/fuse the LiDAR → `image.png`, `cloud.pcd`, `cloud.bag` |
| `record_to_pcd.py` | fuse N static frames from a source record → dense ASCII PCD |
| `pcd_to_bag.py` | PCD → ROS bag; synthesizes a `ring` field for mechanical LiDARs |
| `pick_roi.py` | Open3D interactive board-ROI picker; `--yaml` writes it into the config |
| `overlay_reproj.py` / `render_scene_qa.py` | reprojection overlay (+ colored PCD) for QA |
| `multi_capture.sh` / `pick_multi_roi.sh` | one-command multi-scene runs (auto- / hand-ROI) |

### 1.5 Bring the LiDAR driver online
The LiDAR must publish a point-cloud channel/topic before capture.

**Native ROS LiDAR:** launch the vendor ROS driver; note the `PointCloud2` topic.

**Apollo Cyber RT LiDAR** (this project's case): start the driver **inside the Apollo container,
as the same OS user as the rest of the stack** — e.g. `nvidia`, via `cyber_launch`:
```bash
docker exec -d -u nvidia <apollo_container> bash -lc '
  source /apollo/cyber/setup.bash >/dev/null 2>&1
  export CYBER_IP=127.0.0.1 CYBER_DOMAIN_ID=80 CYBER_PATH=/apollo/cyber
  cd /apollo && cyber_launch start /apollo/modules/drivers/lidar/<driver>/launch/<driver>.launch'
```
> **Critical gotchas learned the hard way:**
> - **User must match.** Starting the driver as `root` creates root-owned shared memory the rest
>   of the (nvidia) stack can't write → `acquire block failed` / `get shm failed: Permission
>   denied`. Kill stray drivers (`pkill -9 -f <driver>/dag`) and remove orphaned root shm
>   (`ipcrm -m <id>`) if you hit this.
> - **Absolute dag path.** cyber_launch resolves a *relative* `<dag_conf>` against `/apollo/cyber`;
>   use an absolute path in the `.launch` (e.g. `/apollo/modules/.../<driver>.dag`).
> - **Unique node name.** Only one driver instance per domain — "duplicated node" means a previous
>   one is still alive.
> - **Match the network conf** (device IP, host IP, MSOP/PTC ports) in the driver's `*.pb.txt`.

Verify data is flowing (use `echo`, not `list` — `list` is flaky for discovery):
```bash
docker exec -u nvidia <apollo_container> bash -lc \
  'source /apollo/cyber/setup.bash>/dev/null 2>&1; export CYBER_DOMAIN_ID=80;
   cyber_channel echo /apollo/sensor/<lidar>/PointCloud2 | grep -m1 frame_id'
```

---

# 2. Capture the data

### 2.1 Camera intrinsics FIRST (mandatory)
The extrinsic is meaningless with wrong intrinsics. Calibrate this exact camera+lens+resolution
with a chessboard/ChArUco (OpenCV or your intrinsic tool) and record
`fx, fy, cx, cy, k1, k2, p1, p2` (FAST-Calib2 uses these 8; k3 is dropped).
- Aim for **avg reprojection error < ~0.3 px**, radial-monotonicity pass, corners covering the frame.
- **Use the camera's own calibration**, not an average of "similar" cameras — focal length varies
  per unit/zoom and directly sets the reconstructed depth (see §5.5).

### 2.2 Physical board setup
Place the board so **both** sensors see it clearly:
- **As close as practical** — a low-line mechanical LiDAR only lands ~(#beams that hit it) scan
  lines on the board.
- **Away from walls** (>~0.3 m) — a wall right behind confuses plane fitting.
- **Keep tilt moderate** for retro-reflective targets on some LiDARs — steep tilt scatters the
  reflective returns off the board plane (see §4.2). Vary board **position** as much as tilt.

### 2.3 Capture one scene
`capture_scene.sh` grabs one RTSP frame and records + fuses the LiDAR into a bag:
```bash
RTSP=rtsp://user:pass@host:554/live APOLLO_C=<apollo_container> \
CH=/apollo/sensor/<lidar>/PointCloud2 TOPIC=/lidar_points FRAME=<lidar_frame> \
RING_FLAG=<--no-ring for dense/solid | empty to synthesize ring> \
  scripts/capture_scene.sh <CAM> <SCENE> 5     # 5 seconds (~ tens of frames fused)
```
Output → `<REPO>/calib_data/<CAM>/<SCENE>/{image.png, cloud.pcd, cloud.bag}`.
**Always eyeball `image.png`** — the whole board (4 markers + 4 rings) must be crisp and unclipped.

**Native ROS alternative:** `rosbag record -O .../cloud.bag <TOPIC> --duration=5` (keeps a real
`ring` field natively), then place a frame at `.../image.png`.

### 2.4 Multi-scene
Repeat §2.3 for **≥3 board poses** (e.g. forward, tilted left, tilted right, and/or moved
left/center/right). The orchestrators in §4.4 automate this.

---

# 3. Convert the data into calibration format

FAST-Calib2 reads a **ROS bag** with `sensor_msgs/PointCloud2` (or Livox `CustomMsg`). If your
source is native ROS, you already have it. For **Apollo/Cyber or other non-ROS sources**,
`capture_scene.sh` runs these two steps for you; here they are explicitly:

### 3.1 Fuse frames → dense PCD
A single frame of a sparse LiDAR is often below FAST-Calib2's point-count guards. The board and
LiDAR are static during capture, so **fuse many frames**:
```bash
python3 scripts/record_to_pcd.py --record-glob '<record_dir>/*' \
        --channel /apollo/sensor/<lidar>/PointCloud2 --out cloud.pcd --max-frames 20
```
> Fusion densifies each scan line **azimuthally**; it cannot add scan lines (fixed by beam count).
> If the board still looks under-sampled, move it closer.

### 3.2 PCD → ROS bag (+ ring for mechanical LiDARs)
```bash
python3 scripts/pcd_to_bag.py --pcd cloud.pcd --bag cloud.bag \
        --topic /lidar_points --frame <lidar_frame> [--no-ring]
```
- **`ring`** = the LiDAR *scan-line index* (NOT the board rings). If the bag has a `ring` field,
  FAST-Calib2 uses its **mechanical** pipeline (walks each scan line for intensity transitions);
  without it, the **solid** pipeline (clusters bright annulus points → fits circles).
- Apollo clouds carry no `ring`. For a **low-line mechanical** LiDAR (e.g. 16-line), `pcd_to_bag`
  **synthesizes** `ring` from each point's elevation vs the nominal beam table → better extraction.
  For a **dense** LiDAR (e.g. 128-line, non-uniform beams), use **`--no-ring`** (solid pipeline).

### 3.3 Data layout expected by the launch files
```
<REPO>/calib_data/<CAM>/<SCENE>/image.png
<REPO>/calib_data/<CAM>/<SCENE>/cloud.bag
<REPO>/config/cameras/<CAM>.yaml          # intrinsics + board geometry + topic + ROI
<REPO>/output/<CAM>/                        # results
```

### 3.4 The per-camera config
Copy the template to `config/cameras/<CAM>.yaml` and fill:
```yaml
# intrinsics (from §2.1)
  fx: ...  fy: ...  cx: ...  cy: ...
  k1: ...  k2: ...  p1: ...  p2: ...
# board geometry — MEASURE your physical board
  marker_size: 0.20             # ArUco side [m]
  delta_width_qr_center: 0.55   # half horizontal marker-center distance
  delta_height_qr_center: 0.35  # half vertical marker-center distance
  delta_width_circles: 0.5      # horizontal ring-center distance
  delta_height_circles: 0.4     # vertical ring-center distance
  circle_radius: 0.12           # annulus centerline radius [m]
  annulus_half_width: 0.025
  min_detected_markers: 3
# lidar
  lidar_topic: "/lidar_points"
  use_auto_lidar_roi: true      # try true first; false + a manual box if it fails (§4.1)
  x_min: ...  x_max: ...  y_min: ...  y_max: ...  z_min: ...  z_max: ...
```

---

# 4. Calibrate

### 4.1 Set the board ROI
FAST-Calib2 must isolate the board points.
- **Auto-ROI** (`use_auto_lidar_roi: true`) works when the board's reflective returns dominate the
  high-intensity points — typical for a **dense** cloud.
- **Manual ROI** — needed when auto-ROI fails (sparse cloud, other retroreflectors, or a wall
  behind the board). Pick a tight box on the board face:
  ```bash
  # writes use_auto_lidar_roi:false + the box straight into the config
  PYTHON=/usr/bin/python3 python3 scripts/pick_roi.py \
      calib_data/<CAM>/<SCENE>/cloud.pcd --yaml config/cameras/<CAM>.yaml
  ```
  Shift-click ≥4 points on the board (intensity-shaded so the rings show), press Q. A tight box
  forces RANSAC onto the board plane instead of a background/wall plane.

### 4.2 Run single-scene
```bash
docker run --rm --net=host \
  -v "<REPO>:/root/calib_ws/src/fast_calib" \
  -v "<REPO>/docker/.ws_build:/root/calib_ws/build" \
  -v "<REPO>/docker/.ws_devel:/root/calib_ws/devel" \
  <IMG> bash -lc "roslaunch fast_calib calib_cam.launch cam:=<CAM> scene:=<SCENE> rviz:=false"
```
(The node prints the result then loops for RViz — wrap in `timeout 90` or Ctrl-C once
`[Result] RMSE` and `Saved four pairs of target centers` appear.) Result →
`output/<CAM>/single_calib_result.txt`.

**A good run shows:** camera `4 centers found`; LiDAR `4 edge/annulus clusters` with concentric
fits at your board's inner/outer radii; `[Result] RMSE: 0.00xx m`.

**Tuning for hard clouds** (edit `src/lidar_detect.hpp`, rebuild):
- *Sparse 16-line, `boundary clusters: 0`* → lower `clusterMechanicalAnnulusBoundaryCloud`'s
  `setMinClusterSize` (80→~20), raise `setClusterTolerance` (0.09→~0.13).
- *Dense non-uniform (AT128), only 2 of 4 clusters* → lower `clusterAnnulusCloud`'s
  `setMinClusterSize` (200→~30), raise tolerance (0.02→~0.06).
- *Bright annulus points "off-plane"/excluded* → tighten the ROI so RANSAC fits the board (best),
  or widen the plane-inlier thresholds (`0.015`/`0.03`).

### 4.3 Run multi-scene (recommended)
Each single-scene run appends 4 center-pairs to `output/<CAM>/circle_center_record.txt`; the joint
step reads the last ≥3:
```bash
# after ≥3 successful single-scene runs:
docker run --rm --net=host -v ... <IMG> \
  bash -lc "roslaunch fast_calib multi_calib_cam.launch cam:=<CAM>"
# -> output/<CAM>/multi_calib_result.txt
```

### 4.4 One-command orchestrators
```bash
# auto-ROI: pauses to reposition the board each angle, captures, calibrates, joint-fits, renders QA
scripts/multi_capture.sh <CAM_BASE> 3 5

# tilted boards / auto-ROI fails: hand-pick a tight ROI per scene (data already captured)
scripts/pick_multi_roi.sh <CAM>
```
Both auto-start the driver (as the right user) and produce `reproj_scene{1..3}.png` +
`colored_scene{1..3}.pcd` from the **joint** extrinsic.

---

# 5. Evaluate the result

### 5.1 Numeric residuals (read the run log)
- **`[Result] RMSE`** — the cross-sensor registration residual (4 LiDAR vs 4 camera centers).
  Expect low mm to ~1 cm; single-scene ~3–7 mm is typical, denser LiDAR → tighter.
- **`[Geometry][LiDAR] max error / RMSE`** — how well the 4 *LiDAR-measured* centers match the
  board's known geometry (the LiDAR's own measurement error, ~mm–cm).
- **`[Geometry][QR]`** — camera geometry; ≈1e-5 mm because the camera centers are constructed from
  the board pose (see §5.5). This is **not** an intrinsics test.
- **Concentric fits** should report your board's radii (e.g. `0.095 / 0.145`).

### 5.2 Reprojection overlay (the primary visual check)
```bash
# single-scene result:
python3 scripts/overlay_reproj.py calib_data/<CAM>/<SCENE> \
        output/<CAM>/single_calib_result.txt output/<CAM>/reproj.png
# from a MULTI result (no intrinsics in that file → pass --config):
python3 scripts/render_scene_qa.py calib_data/<CAM>/<SCENE> \
        output/<CAM>/multi_calib_result.txt --config config/cameras/<CAM>.yaml \
        --overlay output/<CAM>/reproj_<SCENE>.png --colored output/<CAM>/colored_<SCENE>.pcd
```
The LiDAR is projected into the image, colored by intensity (JET). **Calibration is good when the
red = high-reflectivity points land on the 4 white rings** and the scan lines drape correctly over
walls/desk/floor. For multi-scene, verify **every** scene overlay, not just one.

### 5.3 Colored cloud
`colored_*.pcd` = LiDAR points painted with the camera pixel color. Open in CloudCompare/pcl_viewer;
the board pattern (black slab + white rings) should be crisp, not smeared.
> After a multi run, ignore the single-scene `colored_cloud.pcd` / `single_calib_result.txt` — they
> hold whichever scene ran last (possibly a degenerate single-scene SVD). Trust
> `multi_calib_result.txt` + the per-scene `reproj_*`/`colored_*`.

### 5.4 Physical sanity check
`t` is the LiDAR origin in the camera frame (OpenCV axes: +X right, +Y down, +Z forward). Read off
right/left, up/down, forward/behind, and `|t|` (straight-line separation). Cross-check: a board
point's distance to the LiDAR vs to the camera should differ by ~the `t` component along the view.

### 5.5 Intrinsics ↔ board validation (optional, `intrinsic_board_check.py`)
Independently reconstruct the board from an image with the intrinsics and compare distances to
ground truth:
```bash
python3 intrinsic_board_check.py <image.png> fx fy cx cy k1 k2 p1 p2
```
- **(A) markers, per-marker solvePnP** — the robust probe; good intrinsics reconstruct the board to
  **≈0.5%** (a few mm).
- **(B) ring centers, back-projected from detected pixels** — horizontal matches well; a small
  *vertical* deficit can appear on a **tilted** board (the projected-ellipse center ≠ the circle
  center for a tilted circle — a measurement artifact, not the intrinsics).
- **Note:** inter-feature *distances* are largely **insensitive to focal length** on a
  near-fronto-parallel board (lateral X,Y ∝ pixel offset, independent of fx). Focal length instead
  sets the reconstructed **depth**, which is why the *extrinsic translation* is what shifts when you
  correct the intrinsics — validate intrinsics by reprojection error / depth, not board distances.

### 5.6 Acceptance checklist
- [ ] Camera detects 4 markers on every scene; QR geometry error ~1e-5 mm.
- [ ] LiDAR extracts 4 concentric annuli per scene at the correct radii.
- [ ] Cross-sensor RMSE within tolerance (≤ ~1 cm single-scene; tighter multi-scene).
- [ ] Reprojection overlay: red points on the rings for **all** scenes.
- [ ] `|t|` and orientation match the physical mounting.

---

## Appendix — quick end-to-end (Apollo/Cyber example)

```bash
# 0. build + deps (once)
cd <REPO> && docker/build.sh
python3 -m pip install --user cyber_record protobuf==3.19.4 open3d opencv-python numpy

# 1. intrinsics -> config/cameras/<CAM>.yaml   (chessboard, §2.1)

# 2-4. capture 3 angles + calibrate + QA, all in one:
CH=/apollo/sensor/<lidar>/PointCloud2 TOPIC=/lidar_points FRAME=<frame> RING_FLAG=--no-ring \
LIDAR_LAUNCH=/apollo/modules/drivers/lidar/<drv>/launch/<drv>.launch \
  scripts/multi_capture.sh <CAM> 3 5
#   (tilted boards: use scripts/pick_multi_roi.sh <CAM>_<date> instead)

# 5. inspect output/<CAM>_<date>/: multi_calib_result.txt, reproj_scene{1,2,3}.png,
#    colored_scene{1,2,3}.pcd
```
