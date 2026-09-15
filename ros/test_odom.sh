#!/usr/bin/env bash
# Stage-7 Step-1 gate: bring up the MuJoCo base, drive a known command, and
# verify diff_drive_controller odometry + odom->base_link TF respond correctly.
#   docker compose run --rm dev bash -lc 'source /opt/ros/jazzy/setup.bash && \
#     source install/setup.bash && bash ros/test_odom.sh'
set +e
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p /nori/build

ros2 launch nori_bringup bringup.launch.py use_mock_hardware:=false \
  > /nori/build/odom_launch.log 2>&1 &

active=false
for i in $(seq 1 40); do
  if ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active"; then
    active=true; break
  fi
  sleep 1
done

rc=1
if [ "$active" = true ]; then
  python3 "$HERE/_check_odom.py"
  rc=$?
else
  echo "FAIL: diff_drive_controller never became active — see build/odom_launch.log"
fi

pkill -f ros2_control_node; pkill -f robot_state_publisher
pkill -f "ros2 launch"; pkill -f spawner
sleep 2
echo "=== odom test $([ $rc -eq 0 ] && echo PASS || echo FAIL) ==="
exit $rc
