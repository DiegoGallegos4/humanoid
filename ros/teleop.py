#!/usr/bin/env python3
"""Keyboard teleop for Nori's base — drive it through ROS and record the run.

Publishes TwistStamped to /diff_drive_controller/cmd_vel and records
/joint_states + /odom so you can replay the drive in the MuJoCo GUI afterwards
(simulation/mujoco/replay.py).

  Interactive (a TTY):   w/s = forward/back, a/d = turn left/right,
                         space = stop, q = quit & save.
  Non-interactive (piped/CI): runs a short scripted drive instead, so the
                         record path is testable headless.
"""
import math
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

OUT = "/nori/simulation/mujoco/demo_log.npz"
LIN_STEP, ANG_STEP = 0.1, 0.3
LIN_MAX, ANG_MAX = 0.6, 2.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Teleop(Node):
    def __init__(self):
        super().__init__("nori_teleop")
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.names = []
        self.frames = []
        self.base_rows = []
        self.base = (0.0, 0.0, 0.0)
        self.t0 = None
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self._odom, 20)

    def _odom(self, m):
        p = m.pose.pose
        self.base = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _js(self, m):
        if not self.names:
            self.names = list(m.name)
        if self.t0 is None:
            return
        idx = {n: i for i, n in enumerate(m.name)}
        self.frames.append((time.time() - self.t0,
                            [m.position[idx[n]] if n in idx else 0.0 for n in self.names]))
        self.base_rows.append(self.base)

    def publish(self, lin, ang):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        self.pub.publish(msg)

    def save(self):
        if not self.frames:
            print("no frames recorded")
            return
        times = np.array([f[0] for f in self.frames])
        pos = np.array([f[1] for f in self.frames])
        np.savez(OUT, names=np.array(self.names), times=times, positions=pos,
                 base_xyt=np.array(self.base_rows))
        print(f"\nrecorded {len(self.frames)} frames, "
              f"base end x,y,yaw={np.round(self.base_rows[-1], 3)} -> {OUT}")


def wait_for_names(node):
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.names:
            return


def run_interactive(node):
    lin = ang = 0.0
    old = termios.tcgetattr(sys.stdin)
    print("teleop: w/s fwd/back, a/d turn, space stop, q quit")
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.t0 = time.time()
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c == "w":
                    lin = min(LIN_MAX, lin + LIN_STEP)
                elif c == "s":
                    lin = max(-LIN_MAX, lin - LIN_STEP)
                elif c == "a":
                    ang = min(ANG_MAX, ang + ANG_STEP)
                elif c == "d":
                    ang = max(-ANG_MAX, ang - ANG_STEP)
                elif c == " ":
                    lin = ang = 0.0
                elif c == "q":
                    break
                print(f"\r lin={lin:+.2f} ang={ang:+.2f}   ", end="", flush=True)
            node.publish(lin, ang)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.publish(0.0, 0.0)


def run_scripted(node):
    print("no TTY: running scripted drive (forward, turn, forward)")
    sched = [(4.0, 0.4, 0.0), (7.0, 0.0, 1.0), (11.0, 0.4, 0.0), (13.0, 0.0, 0.0)]
    node.t0 = time.time()
    while rclpy.ok():
        t = time.time() - node.t0
        if t >= 13.0:
            break
        lin, ang = next((lx, az) for until, lx, az in sched if t < until)
        node.publish(lin, ang)
        rclpy.spin_once(node, timeout_sec=0.02)
    node.publish(0.0, 0.0)


def main():
    rclpy.init()
    node = Teleop()
    wait_for_names(node)
    if sys.stdin.isatty():
        run_interactive(node)
    else:
        run_scripted(node)
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
