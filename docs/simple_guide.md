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

## 2. Move the code into Apollo and compile

FAST-Calib2 builds as a normal Apollo module. Copy the code into the Apollo
workspace, then run the standard Apollo build — scoped to this module only, so
it does not build the Apollo tree:

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

The first build compiles PCL/OpenCV once (slow); rebuilds are incremental.
All following commands run inside the container from
`/apollo/modules/calibration/fast_calib`, calling the binaries via `$FC/...`.

## 3. Run calibration

Everything lives under the module directory
(`/apollo/modules/calibration/fast_calib`). Full layout for one camera
(`cam0`) with three scenes — copy real files, do **not** symlink (symlinks to
host paths dangle inside the container):

```
config/cameras/cam0.yaml            # per-camera config: copy _template.yaml and
                                    #   fill intrinsics, board geometry,
                                    #   lidar_channel, frames, ROI
calib_data/cam0/scene1/image.png    # camera frame of board placement 1
calib_data/cam0/scene1/record/rec.00000    # cyber record of the LiDAR channel
                                    #   (alternative: cloud.pcd instead of record/)
calib_data/cam0/scene1/cloud_roi.txt       # optional manual ROI (pick_roi.py),
                                    #   auto-applied for this scene
calib_data/cam0/scene2/...          # placement 2  (≥3 scenes total,
calib_data/cam0/scene3/...          #  different board poses)
output/cam0/                        # results are written here
```

Sample configs in the repo: `config/cameras/_template.yaml` is the annotated
skeleton to copy (every `# TODO` must be replaced); `cam0.yaml`–`cam4.yaml`
are real filled-in examples from our 5-camera rig.

Run single-scene calibration:

```bash
$FC/fast_calib --config config/cameras/<cam>.yaml --scene calib_data/<cam>/<scene> --output output/<cam>
```

After collecting at least three scenes (run the command above once per scene),
run multi-scene joint calibration:

```bash
$FC/multi_fast_calib --config config/cameras/<cam>.yaml --output output/<cam>
```

Results in `output/<cam>/`: 
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
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - solid
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/record /apollo/sensor/<lidar>/PointCloud2 solid
```

Run mechanical LiDAR data (set `beam_altitudes_deg` in the config so the
scan-ring index is synthesized for clouds without a `ring` field):

```bash
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - mech
```

The test tool writes into the config's output directory:

- `*_centers.txt` — extracted annulus center coordinates
- `*_debug_cloud.pcd` — board cloud, annulus points, boundary points, and
  center markers for visualization

Debug PCD colors: board points — intensity color map; annulus points — green;
solid-LiDAR boundary points — red; centers — white spheres.
