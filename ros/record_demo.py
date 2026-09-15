#!/usr/bin/env python3
"""Drive a short choreography through ROS and record the resulting joint motion.

Runs inside the dev container while `bringup.launch.py` (MuJoCo mode) is up. It
publishes a multi-point body trajectory + wheel velocities to the real
controllers, records /joint_states throughout, and saves the result to
simulation/mujoco/demo_log.npz. Play it back on the Mac with replay.py.

  ros2 launch nori_bringup bringup.launch.py &   # (wait for controllers active)
  python3 ros/record_demo.py
"""
import time

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

OUT = "/nori/simulation/mujoco/demo_log.npz"
DURATION = 15.0  # seconds to record

# Body joints we choreograph (subset; controller allows partial goals).
BODY = ["torso_lift", "head_pan", "head_tilt",
        "left_shoulder_lift", "left_elbow", "left_wrist_roll",
        "right_shoulder_lift", "right_elbow", "right_wrist_roll"]

# (time_from_start, {joint: target}) — a little wave + a lift up/down.
WALTZ = [
    (1.0,  dict(torso_lift=0.0)),
    (3.0,  dict(torso_lift=0.28, head_pan=0.7, left_shoulder_lift=0.9,
                left_elbow=1.6, right_shoulder_lift=0.9, right_elbow=1.6)),
    (6.0,  dict(head_pan=-0.7, head_tilt=0.4, left_wrist_roll=2.5,
                right_wrist_roll=-2.5)),
    (9.0,  dict(torso_lift=0.10, head_tilt=-0.4, left_elbow=0.3,
                right_elbow=0.3, left_shoulder_lift=-0.5,
                right_shoulder_lift=-0.5)),
    (12.0, {j: 0.0 for j in BODY}),
]


def make_point(targets, t):
    p = JointTrajectoryPoint()
    p.positions = [float(targets.get(j, 0.0)) for j in BODY]
    p.time_from_start.sec = int(t)
    p.time_from_start.nanosec = int((t - int(t)) * 1e9)
    return p


def main():
    rclpy.init()
    node = Node("nori_record_demo")
    body_pub = node.create_publisher(JointTrajectory, "/body_controller/joint_trajectory", 10)
    cmd_pub = node.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)

    names = []
    frames = []      # list of (t, [positions in `names` order])
    base_rows = []   # list of (x, y, yaw) sampled at each joint_states frame
    base = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    t0 = None

    def yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_cb(msg):
        p = msg.pose.pose
        base.update(x=p.position.x, y=p.position.y, yaw=yaw_of(p.orientation))

    def cb(msg):
        nonlocal names, t0
        if not names:
            names = list(msg.name)
        if t0 is None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        row = [msg.position[idx[n]] if n in idx else 0.0 for n in names]
        frames.append((time.time() - t0, row))
        base_rows.append((base["x"], base["y"], base["yaw"]))

    node.create_subscription(JointState, "/joint_states", cb, 50)
    node.create_subscription(Odometry, "/diff_drive_controller/odom", odom_cb, 20)

    # settle: make sure we have joint names before we start
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.05)
        if names:
            break

    # fire the whole body waltz in one multi-point trajectory
    traj = JointTrajectory()
    traj.joint_names = BODY
    traj.points = [make_point(tg, t) for t, tg in WALTZ]
    body_pub.publish(traj)

    t0 = time.time()
    # base tour: (until_t, linear_x m/s, angular_z rad/s) — drive, turn, drive, stop
    drive_sched = [(4.0, 0.4, 0.0), (7.0, 0.0, 1.0), (11.0, 0.4, 0.0),
                   (13.0, 0.0, -1.0), (DURATION, 0.0, 0.0)]
    while rclpy.ok():
        t = time.time() - t0
        if t >= DURATION:
            break
        lin, ang = next((lx, az) for until, lx, az in drive_sched if t < until)
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        cmd_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)

    stop = TwistStamped()
    stop.header.stamp = node.get_clock().now().to_msg()
    cmd_pub.publish(stop)

    times = np.array([f[0] for f in frames])
    pos = np.array([f[1] for f in frames])
    base_xyt = np.array(base_rows)
    np.savez(OUT, names=np.array(names), times=times, positions=pos, base_xyt=base_xyt)
    print(f"recorded {len(frames)} frames over {times[-1]:.1f}s, "
          f"base end x,y,yaw={np.round(base_xyt[-1], 3)} -> {OUT}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
