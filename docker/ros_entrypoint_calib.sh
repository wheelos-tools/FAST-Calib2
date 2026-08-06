#!/usr/bin/env bash
# Source ROS, and the workspace overlay if it has been built yet.
set -e
source /opt/ros/noetic/setup.bash
if [ -f /root/calib_ws/devel/setup.bash ]; then
    source /root/calib_ws/devel/setup.bash
fi
exec "$@"
