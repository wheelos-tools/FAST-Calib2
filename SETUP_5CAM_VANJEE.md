# 5-Camera + Vanjee LiDAR Extrinsic Calibration (FAST-Calib2)

This repo is set up to calibrate **5 cameras** (cam0..cam4) against **one Vanjee
mechanical LiDAR** using FAST-Calib2's reflective-annular-target pipeline.

FAST-Calib2 calibrates **one camera vs one LiDAR per run** and works **offline**
on a recorded `(rosbag, image)` pair. "5 cameras" therefore means running the
same pipeline 5 times — once per camera — each producing its own
`T_cam_lidar` extrinsic relative to the shared Vanjee LiDAR.

## Sensor facts baked into the configs

- **Vanjee LiDAR**: mechanical spinning LiDAR, data (MSOP) endpoint
  `192.168.10.6:3001`. Its ROS driver publishes `sensor_msgs/PointCloud2` with a
  `ring` field, so FAST-Calib2 auto-detects the **mechanical** LiDAR pipeline.
  The recorded topic is assumed to be `/vanjee_points` — confirm/edit in the
  per-camera configs.
- **Cameras**: intrinsics and image topics are **placeholders** (`# TODO`) in
  `config/cameras/camN.yaml`. Fill them before trusting any result.

## Build (containerized — host has no ROS)

The Jetson host has Docker (nvidia runtime, arm64) but no ROS/PCL, so the build
runs in a ROS Noetic container. Nothing is installed on the host.

```bash
cd ~/01code/FAST-Calib2
docker/build.sh          # builds image fast-calib2:noetic + compiles the workspace
```

Compiled binaries persist in `docker/.ws_devel/` (gitignored) across runs.

## Per-camera config

Edit `config/cameras/camN.yaml` for each camera:

1. **Intrinsics** — `fx fy cx cy k1 k2 p1 p2` (replace the `# TODO` placeholders).
2. **LiDAR topic** — confirm `lidar_topic` matches the Vanjee driver's recorded
   PointCloud2 topic.
3. **Target geometry** — match the physical reflective annular board (defaults are
   the repo's example board).

## Data layout

Drop one `(image, cloud)` pair per scene under:

```
calib_data/
  cam0/
    scene1/  { image.png, cloud.bag }
    scene2/  { image.png, cloud.bag }
    scene3/  { image.png, cloud.bag }
  cam1/ ...  cam4/ ...
```

- `image.png` — the camera frame that sees the calibration board.
- `cloud.bag` — a rosbag containing the Vanjee `/vanjee_points` messages for the
  same moment.

(File names `image.png` / `cloud.bag` are what the launch files expect; adjust the
launch args/paths if you use other names.)

## Run

Single scene (prints `T_cam_lidar` + RMSE, then loops for RViz — press Ctrl-C):

```bash
docker/run.sh cam0 scene1        # rviz off by default (headless container)
docker/run.sh cam0 scene1 true   # with RViz (needs X11; see docker/README.md)
```

Multi-scene joint extrinsic (collect >=3 scenes first, then combine):

```bash
docker/run.sh cam0 scene1
docker/run.sh cam0 scene2
docker/run.sh cam0 scene3
docker/run_multi.sh cam0         # reads the last 3 scenes -> multi_calib_result.txt
```

Repeat for cam1..cam4.

## Output

Per camera under `output/<cam>/`:

- single-scene extrinsic + reprojection image + colored cloud
- `circle_center_record.txt` — appended each single-scene run (input to multi-scene)
- `multi_calib_result.txt` — final joint `T_cam_lidar`

## What still needs your input

- Real camera intrinsics for cam0..cam4.
- Confirm the Vanjee recorded topic name (`/vanjee_points` assumed).
- Confirm the calibration-board dimensions if not the repo's example board.
- A working Vanjee ROS driver to record the `cloud.bag` files (outside this repo).
