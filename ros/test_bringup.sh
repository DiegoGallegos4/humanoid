#!/usr/bin/env bash
# Headless Stage-6 smoke test: bring up the control stack, spawn controllers,
# send a trajectory, and confirm /joint_states reflects it. Writes to $OUT and
# always tears down. Runs inside the dev container:
#   docker compose run --rm dev bash -lc 'source /opt/ros/jazzy/setup.bash && \
#     source install/setup.bash && ros/test_bringup.sh <mock|mujoco>'
set +e
MODE="${1:-mock}"
OUT=/nori/build/bringup_test.out
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p /nori/build
: > "$OUT"

MOCK=false; [ "$MODE" = "mock" ] && MOCK=true
ros2 launch nori_bringup bringup.launch.py use_mock_hardware:=$MOCK \
  > /nori/build/launch.log 2>&1 &

# wait (bounded) for body_controller to reach "active"
active=false
for i in $(seq 1 40); do
  if ros2 control list_controllers 2>/dev/null | grep -q "body_controller.*active"; then
    active=true; break
  fi
  sleep 1
done

{
  echo "=== mode: $MODE ==="
  echo "=== controllers ==="
  ros2 control list_controllers 2>/dev/null || echo "(controller_manager unreachable)"
  if [ "$active" = true ]; then
    echo "=== joint_states BEFORE ==="
    python3 "$HERE/_check_states.py" torso_lift head_pan left_elbow
    echo "=== send trajectory (torso_lift=0.2, head_pan=0.5, left_elbow=1.0) ==="
    ros2 topic pub --once /body_controller/joint_trajectory \
      trajectory_msgs/msg/JointTrajectory \
      "{joint_names: [torso_lift, head_pan, left_elbow], points: [{positions: [0.2, 0.5, 1.0], time_from_start: {sec: 1}}]}" >/dev/null 2>&1
    sleep 3
    echo "=== joint_states AFTER ==="
    python3 "$HERE/_check_states.py" torso_lift head_pan left_elbow
  else
    echo "(body_controller never became active — see build/launch.log)"
  fi
} >> "$OUT" 2>&1

# teardown
pkill -f ros2_control_node; pkill -f robot_state_publisher
pkill -f "ros2 launch"; pkill -f spawner; pkill -f rviz2
sleep 2
echo "=== done ===" >> "$OUT"
cat "$OUT"
