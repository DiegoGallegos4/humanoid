#!/usr/bin/env python3
"""Drive the robot to explore, then print the slam_toolbox /map as ASCII.

'#' = occupied (wall/obstacle), '.' = known free, ' ' = unknown. Robot path is
driven here so the map fills in. Run with bringup + slam already up.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def main():
    rclpy.init()
    node = Node("nori_show_map")
    pub = node.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    last = {}
    node.create_subscription(OccupancyGrid, "/map", lambda m: last.update(map=m), qos)

    def drive(lin, ang, dt):
        end = time.time() + dt
        while time.time() < end:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.twist.linear.x = lin
            msg.twist.angular.z = ang
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)

    # explore: spin, then trace a small square so walls are seen from angles
    drive(0.0, 0.6, 11.0)
    for _ in range(4):
        drive(0.25, 0.0, 3.0)
        drive(0.0, 0.9, 1.8)
    drive(0.0, 0.6, 11.0)
    drive(0.0, 0.0, 0.5)
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.05)

    m = last.get("map")
    if m is None:
        print("no /map")
        sys.exit(1)
    w, h = m.info.width, m.info.height
    occ = sum(1 for c in m.data if c >= 65)
    free = sum(1 for c in m.data if 0 <= c < 65)
    print(f"\nmap {w}x{h} @ {m.info.resolution:.3f} m  occ={occ} free={free}\n")
    # rows top-down (grid origin is bottom-left, so print highest row first)
    for row in range(h - 1, -1, -2):            # every other row to fit terminal
        line = []
        for col in range(0, w, 2):
            c = m.data[row * w + col]
            line.append("#" if c >= 65 else ("." if 0 <= c < 65 else " "))
        print("".join(line))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
