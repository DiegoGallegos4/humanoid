#!/usr/bin/env python3
"""Drive the robot so slam_toolbox can map, then check the map + TF are real.

Used by test_slam.sh (bringup + slam already up in MuJoCo mode). Rotates the
base in place to sweep the room, then asserts /map is a non-trivial occupancy
grid (has both occupied and free cells) and the map->odom TF is published.
Exits non-zero on failure so it gates like CI.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

CMD = "/diff_drive_controller/cmd_vel"


def main():
    rclpy.init()
    node = Node("nori_check_slam")
    pub = node.create_publisher(TwistStamped, CMD, 10)

    # /map is latched (transient local) — match its QoS or we get nothing.
    map_qos = QoSProfile(depth=1)
    map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    map_qos.reliability = QoSReliabilityPolicy.RELIABLE
    last = {}
    node.create_subscription(OccupancyGrid, "/map",
                             lambda m: last.update(map=m), map_qos)
    tf_buf = Buffer()
    TransformListener(tf_buf, node)

    def drive(lin, ang, dt):
        end = time.time() + dt
        while time.time() < end:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.twist.linear.x = lin
            msg.twist.angular.z = ang
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)

    # explore: a full spin to see the walls, then trace a square so cells get
    # hit from several angles and firm up as occupied (a lone spin isn't enough)
    drive(0.0, 0.6, 11.0)
    for _ in range(4):
        drive(0.25, 0.0, 3.0)
        drive(0.0, 0.9, 1.8)
    drive(0.0, 0.0, 0.5)
    for _ in range(60):       # let the final map_update tick land
        rclpy.spin_once(node, timeout_sec=0.05)

    # --- test 1: /map is a non-trivial occupancy grid --------------------------
    m = last.get("map")
    if m is None:
        print("FAIL: no /map published")
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    occ = sum(1 for c in m.data if c >= 65)
    free = sum(1 for c in m.data if 0 <= c < 65)
    unknown = sum(1 for c in m.data if c < 0)
    map_ok = occ > 50 and free > 200
    print(f"map:     {m.info.width}x{m.info.height} @ {m.info.resolution:.3f} m  "
          f"occ={occ} free={free} unknown={unknown}  -> {'OK' if map_ok else 'FAIL'}")

    # --- test 2: map->odom TF is published by slam_toolbox ---------------------
    tf_ok = False
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.05)
        if tf_buf.can_transform("map", "odom", rclpy.time.Time()):
            tf_ok = True
            break
    print(f"tf:      map->odom {'present' if tf_ok else 'MISSING'}  "
          f"-> {'OK' if tf_ok else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (map_ok and tf_ok) else 1)


if __name__ == "__main__":
    main()
