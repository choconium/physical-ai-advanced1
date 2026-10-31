#!/usr/bin/env bash
set -eo pipefail
cd /project
source tools/ros_env.sh
mkdir -p artifacts
mkdir -p /root/.gz/sim/8
cp config/gui.config /root/.gz/sim/8/gui.config
children=()
cleanup() { kill -INT "${children[@]}" 2>/dev/null || true; wait || true; }
trap cleanup EXIT INT TERM
Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset >artifacts/xvfb.log 2>&1 & children+=("$!")
for ((i=0; i<50; i++)); do xdpyinfo -display :99 >/dev/null 2>&1 && break; sleep .1; done
fluxbox >artifacts/fluxbox.log 2>&1 & children+=("$!")
ros2 run rmw_zenoh_cpp rmw_zenohd >artifacts/zenoh.log 2>&1 & children+=("$!")
sleep 2
ros2 launch ffw_bringup ffw_sg2_follower_ai_gazebo.launch.py rviz:=false \
  >artifacts/gazebo.log 2>&1 & children+=("$!")
sleep 15
xdotool search --name 'Gazebo Sim' windowsize 1280 690 windowmove 0 0 || true
# model:=sg2 is essential: the vendor launch otherwise defaults to BG2.
ros2 launch ffw_moveit_config moveit.launch.py model:=sg2 use_sim:=true start_rviz:=false \
  >artifacts/moveit.log 2>&1 & children+=("$!")
python3 tools/configure_view.py --camera -.25 -.95 1.40 --target .45 -.28 1.05
wait
