#!/usr/bin/env bash
# Record a SLAM mapping run for rendering (PNG + MP4) on the Mac.
#   docker compose run --rm dev bash -lc 'source /opt/ros/jazzy/setup.bash && \
#     source install/setup.bash && bash ros/record_map.sh'
# then on the Mac:  .venv/bin/python simulation/mujoco/render_map.py
set +e
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p /nori/build

ros2 launch nori_bringup bringup.launch.py use_mock_hardware:=false \
  > /nori/build/record_map_bringup.log 2>&1 &
for i in $(seq 1 40); do
  ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" && break
  sleep 1
done

ros2 launch nori_navigation slam.launch.py > /nori/build/record_map_slam.log 2>&1 &
for i in $(seq 1 30); do
  ros2 topic list 2>/dev/null | grep -q "^/map$" && break
  sleep 1
done

python3 "$HERE/record_map.py"

pkill -f async_slam_toolbox_node
pkill -f ros2_control_node; pkill -f robot_state_publisher
pkill -f "ros2 launch"; pkill -f spawner
sleep 2
