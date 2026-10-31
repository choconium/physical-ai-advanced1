#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=64
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 QT_QPA_PLATFORM=xcb
export PYTHONPATH=/project/src:${PYTHONPATH:-}
if (( $# )); then exec "$@"; fi
