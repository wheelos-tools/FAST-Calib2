# FAST-Calib2 Docker (ROS Noetic, aarch64)

The Jetson host has no ROS/PCL, so FAST-Calib2 builds and runs inside a ROS
Noetic container. The host is untouched.

## Files

- `Dockerfile` — `ros:noetic-perception` + PCL/OpenCV/Eigen build deps.
- `ros_entrypoint_calib.sh` — sources ROS and the workspace overlay.
- `build.sh` — builds the image and compiles the catkin workspace.
- `run.sh <cam> <scene> [rviz]` — single-scene calibration.
- `run_multi.sh <cam>` — multi-scene joint calibration.

## How the workspace is mounted

The repo is bind-mounted into a catkin workspace at
`/root/calib_ws/src/fast_calib`. Build/devel trees are bind-mounted from
`docker/.ws_build` and `docker/.ws_devel` so compiled binaries persist across
`docker run --rm`.

## Build

```bash
docker/build.sh
```

## Run

```bash
docker/run.sh cam0 scene1          # headless (no RViz)
docker/run_multi.sh cam0
```

The node prints `T_cam_lidar` and RMSE, then enters a publish loop for RViz —
press **Ctrl-C** once the result is printed.

## RViz (optional)

RViz needs an X server. On the Jetson desktop:

```bash
xhost +local:root
docker/run.sh cam0 scene1 true
```

`run.sh` forwards `$DISPLAY` and `/tmp/.X11-unix` when the 3rd arg is `true`.
For headless use, leave RViz off — the extrinsic result is printed to stdout and
saved under `output/<cam>/` regardless.
