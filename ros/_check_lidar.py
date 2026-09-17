#!/usr/bin/env python3
"""Check the simulated 2D lidar publishes a sane LaserScan + its TF frame.

Used by test_lidar.sh (bringup must already be up in MuJoCo mode). The robot
starts at the centre of the 6x6 m room in scene.xml, so a 360 deg fan should
see a wall on every beam, all within the room's half-diagonal (~4.25 m) and
none closer than the chassis. Exits non-zero on failure so it gates like CI.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SCAN = "/scan"


def main():
    rclpy.init()
    node = Node("nori_check_lidar")
    last = {}
    node.create_subscription(LaserScan, SCAN, lambda m: last.update(scan=m),
                             rclpy.qos.qos_profile_sensor_data)
    tf_buf = Buffer()
    TransformListener(tf_buf, node)

    # --- test 1: a LaserScan arrives -------------------------------------------
    scan = None
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if "scan" in last:
            scan = last["scan"]
            break
    if scan is None:
        print("FAIL: no LaserScan on", SCAN)
        sys.exit(1)

    n = len(scan.ranges)
    finite = [r for r in scan.ranges if math.isfinite(r)]
    frac = len(finite) / n if n else 0.0
    # closed room: essentially every beam hits a wall
    count_ok = n >= 90 and frac >= 0.9
    print(f"scan:    {n} beams, {frac*100:.0f}% return a range  "
          f"-> {'OK' if count_ok else 'FAIL'}")

    # --- test 2: ranges are physically sane (no self-hit, inside the room) -----
    rmin, rmax = min(finite), max(finite)
    range_ok = rmin > 0.3 and rmax < 5.0
    print(f"range:   min {rmin:.2f} m, max {rmax:.2f} m (room ~3-4.25 m)  "
          f"-> {'OK' if range_ok else 'FAIL'}")

    # --- test 3: the scan's TF frame is published ------------------------------
    frame = scan.header.frame_id
    tf_ok = False
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.05)
        if tf_buf.can_transform("base_link", frame, rclpy.time.Time()):
            tf_ok = True
            break
    print(f"tf:      base_link->{frame} {'present' if tf_ok else 'MISSING'}  "
          f"-> {'OK' if tf_ok else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (count_ok and range_ok and tf_ok) else 1)


if __name__ == "__main__":
    main()
