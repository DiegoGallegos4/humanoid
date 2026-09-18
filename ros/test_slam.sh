#!/usr/bin/env bash
# Stage-7 Step-4 gate: bring up the MuJoCo base + slam_toolbox, drive the robot
# to sweep the room, and verify a real occupancy map + map->odom TF are built.
#   docker compose run --rm dev bash -lc 'source /opt/ros/jazzy/setup.bash && \
#     source install/setup.bash && bash ros/test_slam.sh'
set +e
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p /nori/build

ros2 launch nori_bringup bringup.launch.py use_mock_hardware:=false \
  > /nori/build/slam_bringup.log 2>&1 &

active=false
for i in $(seq 1 40); do
  if ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active"; then
    active=true; break
  fi
  sleep 1
done

rc=1
if [ "$active" = true ]; then
  ros2 launch nori_navigation slam.launch.py > /nori/build/slam_node.log 2>&1 &
  # wait for slam_toolbox to advertise /map
  for i in $(seq 1 30); do
    ros2 topic list 2>/dev/null | grep -q "^/map$" && break
    sleep 1
  done
  python3 "$HERE/_check_slam.py"
  rc=$?
else
  echo "FAIL: diff_drive_controller never became active — see build/slam_bringup.log"
fi

pkill -f async_slam_toolbox_node
pkill -f ros2_control_node; pkill -f robot_state_publisher
pkill -f "ros2 launch"; pkill -f spawner
sleep 2
echo "=== slam test $([ $rc -eq 0 ] && echo PASS || echo FAIL) ==="
exit $rc
