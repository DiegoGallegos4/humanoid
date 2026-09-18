#!/usr/bin/env python3
"""Grab one /scan and draw it as a top-down ASCII map — a quick eyeball test
that the simulated lidar actually sees the room.

Robot is 'R' at the centre, facing up (+x forward, +y left). '#' = a lidar
return (a wall or obstacle hit). Run with bringup already up in MuJoCo mode.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

N = 41          # grid is N x N cells
RES = 0.2       # metres per cell -> covers +/- 4 m
C = N // 2


def main():
    rclpy.init()
    node = Node("nori_show_scan")
    last = {}
    node.create_subscription(LaserScan, "/scan", lambda m: last.update(scan=m),
                             rclpy.qos.qos_profile_sensor_data)
    scan = None
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if "scan" in last:
            scan = last["scan"]
            break
    if scan is None:
        print("no /scan received")
        return

    grid = [[" "] * N for _ in range(N)]
    hits = 0
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r):
            continue
        a = scan.angle_min + i * scan.angle_increment
        x, y = r * math.cos(a), r * math.sin(a)          # lidar frame
        row = C - int(round(x / RES))                    # +x forward -> up
        col = C - int(round(y / RES))                    # +y left -> left
        if 0 <= row < N and 0 <= col < N:
            grid[row][col] = "#"
            hits += 1
    grid[C][C] = "R"

    print(f"\n/scan: {len(scan.ranges)} beams, {hits} plotted "
          f"(cell = {RES} m, view +/-{C*RES:.1f} m, R = robot, up = forward)\n")
    border = "+" + "-" * (N * 2) + "+"
    print(border)
    for row in grid:
        print("|" + "".join(c + " " for c in row) + "|")
    print(border)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
