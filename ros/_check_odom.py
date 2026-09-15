#!/usr/bin/env python3
"""Drive the base a known command and check odometry + TF respond correctly.

Used by test_odom.sh (bringup must already be up in MuJoCo mode). Exits non-zero
if any check fails so it works as a CI-style gate.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

CMD = "/diff_drive_controller/cmd_vel"
ODOM = "/diff_drive_controller/odom"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node("nori_check_odom")
    pub = node.create_publisher(TwistStamped, CMD, 10)
    last = {}
    node.create_subscription(Odometry, ODOM, lambda m: last.update(pose=m.pose.pose), 10)
    tf_buf = Buffer()
    TransformListener(tf_buf, node)

    def spin(dt):
        end = time.time() + dt
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def drive(lin, ang, dt):
        end = time.time() + dt
        while time.time() < end:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.twist.linear.x = lin
            msg.twist.angular.z = ang
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)

    def pose():
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.02)
            if "pose" in last:
                p = last["pose"]
                return p.position.x, p.position.y, yaw_of(p.orientation)
        return None

    spin(1.0)
    p0 = pose()
    if p0 is None:
        print("FAIL: no odometry on", ODOM)
        sys.exit(1)

    # --- test 1: forward 0.3 m/s for 2 s -> expect ~0.6 m of +x travel ---------
    drive(0.3, 0.0, 2.0)
    drive(0.0, 0.0, 0.3)
    p1 = pose()
    dx = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    fwd_ok = 0.35 <= dx <= 0.75  # allow wheel slip / accel ramp
    print(f"forward: moved {dx:.3f} m (want ~0.6)  -> {'OK' if fwd_ok else 'FAIL'}")

    # --- test 2: rotate 1.0 rad/s for 2 s -> expect ~2 rad of yaw, little travel -
    drive(0.0, 1.0, 2.0)
    drive(0.0, 0.0, 0.3)
    p2 = pose()
    dyaw = abs(math.atan2(math.sin(p2[2] - p1[2]), math.cos(p2[2] - p1[2])))
    move_while_turning = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    rot_ok = dyaw >= 1.0 and move_while_turning < 0.2
    print(f"rotate:  yaw {dyaw:.3f} rad, drift {move_while_turning:.3f} m  "
          f"-> {'OK' if rot_ok else 'FAIL'}")

    # --- test 3: odom -> base_link TF is published -----------------------------
    tf_ok = False
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.05)
        if tf_buf.can_transform("odom", "base_link", rclpy.time.Time()):
            tf_ok = True
            break
    print(f"tf:      odom->base_link {'present' if tf_ok else 'MISSING'}  "
          f"-> {'OK' if tf_ok else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (fwd_ok and rot_ok and tf_ok) else 1)


if __name__ == "__main__":
    main()
