#!/usr/bin/env bash
# Stage-7 Step-3 gate: bring up the MuJoCo base and verify the simulated 2D
# lidar publishes a sane /scan in a TF frame (walls detected, no self-hits).
#   docker compose run --rm dev bash -lc 'source /opt/ros/jazzy/setup.bash && \
#     source install/setup.bash && bash ros/test_lidar.sh'
set +e
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p /nori/build

ros2 launch nori_bringup bringup.launch.py use_mock_hardware:=false \
  > /nori/build/lidar_launch.log 2>&1 &

active=false
for i in $(seq 1 40); do
  if ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active"; then
    active=true; break
  fi
  sleep 1
done

rc=1
if [ "$active" = true ]; then
  python3 "$HERE/_check_lidar.py"
  rc=$?
else
  echo "FAIL: diff_drive_controller never became active — see build/lidar_launch.log"
fi

pkill -f ros2_control_node; pkill -f robot_state_publisher
pkill -f "ros2 launch"; pkill -f spawner
sleep 2
echo "=== lidar test $([ $rc -eq 0 ] && echo PASS || echo FAIL) ==="
exit $rc
