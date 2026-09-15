#!/usr/bin/env python3
"""Subscribe to /joint_states, wait for one message, print the requested joints.
Used by test_bringup.sh. Usage: _check_states.py joint1 joint2 ..."""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    wanted = sys.argv[1:] or ["torso_lift", "head_pan", "left_elbow"]
    rclpy.init()
    node = Node("nori_state_check")
    got = {}

    def cb(msg):
        got.update(dict(zip(msg.name, msg.position)))
    node.create_subscription(JointState, "/joint_states", cb, 10)

    for _ in range(200):  # ~10 s
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(j in got for j in wanted):
            break
    for j in wanted:
        v = got.get(j)
        print(f"  {j:16s} = {v:+.3f}" if v is not None else f"  {j:16s} = MISSING")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
