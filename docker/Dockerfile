# FAST-Calib2 build/run environment for the Jetson (aarch64).
# ROS Noetic ships PCL 1.10 + OpenCV 4.x, matching CMakeLists requirements,
# without touching the JetPack host. The `perception` variant already bundles
# pcl_ros, pcl_conversions, cv_bridge and OpenCV.
#
# Docker Hub is firewalled on this network, so the base image is pulled through
# a mirror by default. Override with --build-arg REGISTRY=... (e.g. empty for a
# direct Docker Hub pull, or another reachable mirror).
ARG REGISTRY=docker.m.daocloud.io/
FROM ${REGISTRY}library/ros:noetic-perception

# Extra build deps not guaranteed in the base image.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libpcl-dev \
        libopencv-dev \
        libeigen3-dev \
        ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*

# catkin workspace layout. The repo is bind-mounted at src/fast_calib and the
# build/devel trees are bind-mounted from the host so compiled binaries persist
# across `docker run --rm` invocations (see docker/build.sh / docker/run.sh).
ENV CATKIN_WS=/root/calib_ws
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}

COPY ros_entrypoint_calib.sh /ros_entrypoint_calib.sh
RUN chmod +x /ros_entrypoint_calib.sh
ENTRYPOINT ["/ros_entrypoint_calib.sh"]
CMD ["bash"]
