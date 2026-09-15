#!/usr/bin/env bash
# Interactive teleop: bring up the MuJoCo base, then drive it from the keyboard.
# Run in an interactive container shell:
#   docker compose run --rm dev bash -lc 'source /opt/ros/jazzy/setup.bash && \
#     source install/setup.bash && bash ros/drive.sh'
# On quit it saves the run to simulation/mujoco/demo_log.npz — replay on the Mac:
#   .venv/bin/mjpython simulation/mujoco/replay.py
set +e
mkdir -p /nori/build

ros2 launch nori_bringup bringup.launch.py use_mock_hardware:=false \
  > /nori/build/drive_launch.log 2>&1 &

for i in $(seq 1 40); do
  ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" && break
  sleep 1
done

python3 /nori/ros/teleop.py

pkill -f ros2_control_node; pkill -f robot_state_publisher
pkill -f "ros2 launch"; pkill -f spawner
sleep 2
